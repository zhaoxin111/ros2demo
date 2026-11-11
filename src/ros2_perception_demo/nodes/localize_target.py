#!/usr/bin/env python3
"""同步 RGB-D 消息，检测红色目标，再把相机中的三维位置转换到地图坐标系。"""

from collections import deque
from time import monotonic

from geometry_msgs.msg import Point, PointStamped
from message_filters import Subscriber, TimeSynchronizer
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def detect_point(rgb, depth, info):
    """根据红色区域及深度返回光学坐标系中的 PointStamped，无目标时返回 None。

    rgb、depth 为已校正且像素对齐的 rgb8、16UC1 图像；深度单位为毫米。
    info 提供对应的 CameraInfo，三条消息应已按采集时间同步。
    输入格式或内参不符合要求时抛出 ValueError，由节点回调负责记录提示。
    """
    width, height = rgb.width, rgb.height
    # 分开检查坐标系、编码、尺寸和内存布局，便于按报错定位输入问题。
    if (
        not rgb.header.frame_id
        or rgb.header.frame_id != depth.header.frame_id
        or rgb.header.frame_id != info.header.frame_id
    ):
        raise ValueError('RGB、深度和 CameraInfo 必须使用同一个非空光学坐标系')
    if rgb.encoding != 'rgb8' or depth.encoding != '16UC1':
        raise ValueError('需要 rgb8 彩色图和 16UC1 毫米深度图')
    if (
        (depth.width, depth.height) != (width, height)
        or (info.width, info.height) != (width, height)
        or width == 0 or height == 0
    ):
        raise ValueError('RGB、深度和 CameraInfo 的尺寸必须一致且非空')
    if rgb.step < width * 3 or depth.step < width * 2:
        raise ValueError('图像每行的字节数 step 小于像素编码所需长度')
    if len(rgb.data) < rgb.step * height or len(depth.data) < depth.step * height:
        raise ValueError('图像数据长度不足，无法按声明的尺寸和 step 读取')

    # P 为按行展开的 3×4 投影矩阵。此 Demo 使用单目校正图像，P 的平移项为零。
    # fx、fy 是像素焦距；cx、cy 是主点，通常接近图像中心。
    fx, fy, cx, cy = info.p[0], info.p[5], info.p[2], info.p[6]
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
        raise ValueError('CameraInfo.p 需要有限的内参值，且 fx、fy 必须大于零')

    # 直接把消息缓冲区解释为数组，避免逐像素循环。
    # strides 指明“下一行、下一列、下一通道”各跳过多少字节。
    # 不能简单 reshape：真实相机可能在每行末尾添加填充字节，此时 step 会更大。
    pixels = np.ndarray((height, width, 3), dtype=np.uint8,
                        buffer=rgb.data, strides=(rgb.step, 3, 1))
    if depth.is_bigendian:
        depth_dtype = '>u2'
    else:
        depth_dtype = '<u2'
    depths = np.ndarray((height, width), dtype=depth_dtype,
                        buffer=depth.data, strides=(depth.step, 2))

    # 用颜色阈值替代复杂的识别模型：红通道较强，绿、蓝通道较弱。
    red_channel = pixels[:, :, 0]
    green_channel = pixels[:, :, 1]
    blue_channel = pixels[:, :, 2]
    red_mask = (red_channel > 180) & (green_channel < 80) & (blue_channel < 80)
    valid_target_mask = red_mask & (depths > 0)
    rows, columns = np.nonzero(valid_target_mask)
    if rows.size == 0:
        return None

    # 有效像素的平均列、行作为中心；深度中位数能减小个别异常深度值的影响。
    u = float(columns.mean())
    v = float(rows.mean())
    z = float(np.median(depths[rows, columns])) * 0.001
    # 针孔模型反投影：已知像素 (u, v) 和深度 z，求出右、下、前三个方向的米数。
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return PointStamped(header=rgb.header, point=Point(x=x, y=y, z=z))


class LocalizeTarget(Node):
    """把图像处理与 TF 等待分成两个回调，避免等待 TF 时阻塞消息接收。"""

    def __init__(self):
        """创建同步订阅、TF 缓冲区、结果发布器和等待队列。"""
        super().__init__('localize_target')
        self.declare_parameter('tf_wait_sec', 0.5)
        self.declare_parameter('use_latest_tf', False)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Listener 持续接收 /tf、/tf_static，Buffer 保存变换并按时间查询。
        # 队列最多保留 30 帧；处理积压时自动丢弃最旧的观测，限制内存占用。
        self.pending_points = deque(maxlen=30)
        self.camera_pub = self.create_publisher(PointStamped, '/target/point_camera', 10)
        self.map_pub = self.create_publisher(PointStamped, '/target/point_map', 10)
        self.marker_pub = self.create_publisher(Marker, '/target/marker', 10)

        # 三条消息只有 header.stamp 完全相同才触发 on_rgbd，避免把不同帧混用。
        self.streams = [
            Subscriber(self, Image, '/camera/color/image_raw', qos_profile_sensor_data),
            Subscriber(self, Image, '/camera/depth/image_raw', qos_profile_sensor_data),
            Subscriber(self, CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data),
        ]
        self.sync = TimeSynchronizer(self.streams, queue_size=10)
        self.sync.registerCallback(self.on_rgbd)
        # 重试使用稳定时钟；即使 rosbag 暂停 /clock，仍能处理等待超时。
        self.retry_timer = self.create_timer(
            0.02, self.publish_pending, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def on_rgbd(self, rgb, depth, info):
        """同步回调：检测本帧目标，发布相机坐标，并排队等待地图变换。"""
        try:
            point = detect_point(rgb, depth, info)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        if point is None:
            return
        # TF2 把时间零解释为“最新”，因此拒绝缺失采集时间的输入，避免用错位姿。
        if point.header.stamp.sec == 0 and point.header.stamp.nanosec == 0:
            self.get_logger().warning('忽略没有采集时间戳的观测', throttle_duration_sec=2.0)
            return
        self.camera_pub.publish(point)
        # monotonic() 只用于计算真实等待时长，不受系统校时或回放时间跳变影响。
        wait_sec = self.get_parameter('tf_wait_sec').value
        deadline = monotonic() + wait_sec
        self.pending_points.append((point, deadline))

    def publish_pending(self):
        """定时回调：依次转换队首观测；TF 未到时稍后重试，超时则丢弃该帧。"""
        while self.pending_points:
            point, deadline = self.pending_points[0]
            stamp = Time.from_msg(point.header.stamp)
            if self.get_parameter('use_latest_tf').value:
                # 教学开关：时间零会使用最新 TF，可用于观察延迟图像导致的定位偏差。
                stamp = Time()
            try:
                # 参数顺序是“目标坐标系、源坐标系、采集时间”；默认查询不阻塞等待。
                transform = self.tf_buffer.lookup_transform('map', point.header.frame_id, stamp)
            except TransformException as error:
                if monotonic() < deadline:
                    # 立即交还执行权，让 TF 订阅回调有机会填充 Buffer；不能在这里 sleep。
                    return
                self.get_logger().warning(f'等待 TF 超时，丢弃该帧地图观测：{error}',
                                          throttle_duration_sec=2.0)
                self.pending_points.popleft()
                continue

            self.pending_points.popleft()
            map_point = do_transform_point(point, transform)
            # 坐标变换改变位置和 frame_id，但观测仍属于原来的采集时刻。
            map_point.header.stamp = point.header.stamp
            self.map_pub.publish(map_point)
            self.publish_marker(map_point)

    def publish_marker(self, map_point):
        """把地图中的 PointStamped 显示为 RViz 红球；此方法只发布消息。"""
        # 相同的 ns 和 id 会更新原有标记，避免每帧生成一个新的球。
        marker = Marker(header=map_point.header, ns='red_target', id=0, type=Marker.SPHERE)
        marker.pose.position = map_point.point
        marker.pose.orientation.w = 1.0  # 单位四元数，表示不旋转。
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.14
        marker.color.b = 0.10
        marker.color.a = 1.0  # alpha=1 表示完全不透明。
        # 0.7 秒不更新就消失，避免目标已经丢失但 RViz 仍一直显示旧位置。
        marker.lifetime.nanosec = 700_000_000
        self.marker_pub.publish(marker)


def main():
    """启动节点并调度订阅、定时器回调；退出时释放资源。"""
    rclpy.init()
    node = LocalizeTarget()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
