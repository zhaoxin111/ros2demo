#!/usr/bin/env python3
"""模拟移动的 RGB-D 相机，发布图像、相机内参及坐标系之间的 TF。"""

from collections import deque
from math import cos, sin

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


# 光学坐标系约定：x 向右、y 向下、z 向前，单位为米。
OPTICAL_FRAME = 'camera_optical_frame'
WIDTH, HEIGHT = 160, 120
# 针孔相机内参：焦距和图像中心的位置，单位为像素。
FOCAL_LENGTH = 110.0
CX, CY = (WIDTH - 1) / 2, (HEIGHT - 1) / 2
# 目标固定在地图中；相机固定安装在底座前方 0.20 m、上方 0.50 m。
TARGET_MAP = (2.0, 0.25, 0.75)
CAMERA_OFFSET = (0.20, 0.0, 0.50)
PERIOD_SEC = 0.10
# 每 0.10 秒采集一帧，延迟两帧发布，模拟约 0.20 秒的处理延迟。
DELAY_FRAMES = 2


def make_transform(stamp, parent, child, translation, rotation):
    """返回描述子坐标系在父坐标系中位姿的 TransformStamped。

    stamp 为 ROS 时间戳；parent、child 为坐标系名称。
    translation 按 (x, y, z) 传入米数，rotation 按 (x, y, z, w) 传入四元数。
    TF2 使用这条消息把子坐标系中的点转换到父坐标系。
    """
    transform = TransformStamped()
    transform.header = Header(stamp=stamp, frame_id=parent)
    transform.child_frame_id = child
    vector = transform.transform.translation
    vector.x, vector.y, vector.z = translation
    quaternion = transform.transform.rotation
    quaternion.x, quaternion.y, quaternion.z, quaternion.w = rotation
    return transform


class MockRgbd(Node):
    """用定时器模拟采集，用发布器发送三种相机消息及 TF。"""

    def __init__(self):
        """创建 ROS 通信对象和采集缓存，并发布固定的相机安装位姿。"""
        super().__init__('mock_rgbd')
        self.declare_parameter('target_visible', True)
        # 三个相机话题采用相同的传感器 QoS（best_effort）。订阅端也要兼容此设置。
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
        self.dynamic_tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.start_time = self.get_clock().now()
        stamp = self.start_time.to_msg()
        # 底座到相机的关系不随时间变化，所以只需通过 /tf_static 广播一次。
        # 第二条变换把相机的“前、左、上”轴转为光学坐标系的“右、下、前”。
        self.static_tf.sendTransform([
            make_transform(stamp, 'base_link', 'camera_link', CAMERA_OFFSET, (0.0, 0.0, 0.0, 1.0)),
            make_transform(stamp, 'camera_link', OPTICAL_FRAME,
                           (0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5)),
        ])
        # 行坐标为 HEIGHT×1，列坐标为 1×WIDTH。NumPy 广播可由它们计算整幅图的圆形区域。
        self.rows, self.columns = np.ogrid[:HEIGHT, :WIDTH]
        self.pending_captures = deque()
        self.capture_timer = self.create_timer(PERIOD_SEC, self.capture)

    def capture(self):
        """定时回调：计算当前位姿和目标像素，再发布延迟队列中的旧帧。"""
        now = self.get_clock().now()
        stamp = now.to_msg()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        # 底座沿地图 x 方向往复移动，同时小幅左右转动；yaw 的单位是弧度。
        base_x = 0.45 * sin(1.3 * elapsed)
        yaw = 0.16 * sin(1.1 * elapsed)
        self.dynamic_tf.sendTransform(make_transform(
            stamp, 'map', 'base_link', (base_x, 0.0, 0.0),
            (0.0, 0.0, sin(yaw / 2), cos(yaw / 2)),
        ))

        # 第一步：把“地图中的目标位置”转换为“底座中的目标位置”。
        # 先减去底座平移，再旋转 -yaw；最后减去相机安装偏移，得到相机中的位置。
        dx = TARGET_MAP[0] - base_x
        dy = TARGET_MAP[1]
        forward = cos(yaw) * dx + sin(yaw) * dy - CAMERA_OFFSET[0]
        left = -sin(yaw) * dx + cos(yaw) * dy - CAMERA_OFFSET[1]
        up = TARGET_MAP[2] - CAMERA_OFFSET[2]
        # 第二步：针孔投影。光学 x=-left、y=-up、z=forward，因此下面是减号。
        # 当前运动范围保证目标始终在相机前方，即 forward > 0。
        u = CX - FOCAL_LENGTH * left / forward
        v = CY - FOCAL_LENGTH * up / forward
        visible = self.get_parameter('target_visible').value
        self.pending_captures.append((stamp, u, v, forward, visible))

        # TF 立即广播，图像稍后发送，但仍保留采集时间，而不是改成发送时间。
        # 这样可以观察“按采集时间查询 TF”与“使用最新 TF”的差别。
        if len(self.pending_captures) > DELAY_FRAMES:
            capture_stamp, pixel_u, pixel_v, depth_m, target_visible = self.pending_captures.popleft()
            self.publish_capture(capture_stamp, pixel_u, pixel_v, depth_m, target_visible)

    def publish_capture(self, stamp, u, v, depth_m, visible):
        """发布一次采集的 RGB、深度和 CameraInfo，三者共用时间戳与光学坐标系。

        u、v 是目标中心的像素坐标；depth_m 是沿光学 z 轴的深度（米）。
        visible 为 False 时只发布背景，用于练习无目标、查询过期和 Action 超时。
        此方法通过话题发送消息，不返回数据。
        """
        rgb = np.full((HEIGHT, WIDTH, 3), (30, 39, 50), dtype=np.uint8)
        # <u2 表示小端无符号 16 位整数，与深度消息的 16UC1、is_bigendian=False 一致。
        # 深度图使用毫米；背景设为零，表示没有有效深度。
        depth = np.zeros((HEIGHT, WIDTH), dtype='<u2')
        if visible:
            # 圆内像素满足 (列-u)² + (行-v)² <= 半径²；用红色圆斑代表目标。
            distance_squared = (self.columns - round(u)) ** 2 + (self.rows - round(v)) ** 2
            target_mask = distance_squared <= 8 ** 2
            rgb[target_mask] = (235, 32, 42)
            depth[target_mask] = round(depth_m * 1000)

        header = Header(stamp=stamp, frame_id=OPTICAL_FRAME)
        # step 是每行占用的字节数：RGB 每像素 3 字节，深度每像素 2 字节。
        self.rgb_pub.publish(Image(
            header=header, height=HEIGHT, width=WIDTH, encoding='rgb8',
            step=WIDTH * 3, data=rgb.tobytes(),
        ))
        self.depth_pub.publish(Image(
            header=header, height=HEIGHT, width=WIDTH, encoding='16UC1',
            step=WIDTH * 2, data=depth.tobytes(),
        ))
        # K 是原始相机内参矩阵，R 是校正旋转，P 是校正后的投影矩阵。
        # 模拟相机无畸变、不需要额外校正，因此 K 和 P 中使用同一组焦距、中心。
        self.info_pub.publish(CameraInfo(
            header=header, height=HEIGHT, width=WIDTH, distortion_model='plumb_bob',
            d=[0.0] * 5,
            k=[FOCAL_LENGTH, 0.0, CX, 0.0, FOCAL_LENGTH, CY, 0.0, 0.0, 1.0],
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[FOCAL_LENGTH, 0.0, CX, 0.0, 0.0, FOCAL_LENGTH, CY, 0.0, 0.0, 0.0, 1.0, 0.0],
        ))


def main():
    """初始化 ROS 并处理回调；退出时销毁节点、释放 ROS 资源。"""
    rclpy.init()
    node = MockRgbd()
    try:
        # spin 持续调度定时器等回调；没有它，节点不会持续采集。
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
