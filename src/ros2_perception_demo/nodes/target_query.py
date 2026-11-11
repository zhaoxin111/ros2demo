#!/usr/bin/env python3
"""用 Service 查询最近位置，用 Action 收集多帧位置并返回平均值。"""

from time import monotonic

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from ros2_perception_interfaces.action import ObserveTarget
from ros2_perception_interfaces.srv import GetTarget


class Observation:
    """保存一个 Action 任务的状态，便于多个回调共同推进同一任务。"""

    def __init__(self, goal: ServerGoalHandle):
        """从已接受的目标创建任务；goal 中包含所需帧数和超时秒数。"""
        self.goal = goal
        # Future 表示“稍后才有的结果”：执行回调等待它，其他回调负责填入结果。
        self.result = Future()
        # Action 超时按真实流逝时间计算，避免回放暂停后永远无法超时。
        self.deadline = monotonic() + goal.request.timeout_sec
        # 每个任务各自创建一个列表，仅收集接受目标后收到的地图位置。
        self.samples = []


def calculate_mean_position(samples):
    """返回非空 PointStamped 列表的平均位置，使用最后一帧的时间戳和坐标系。

    调用方需保证所有样本都在 map 坐标系中；这里的平均值是多帧统计结果，
    不是一次新的相机采集，因此沿用最后一帧的 Header。
    """
    total_x = 0.0
    total_y = 0.0
    total_z = 0.0
    for sample in samples:
        total_x += sample.point.x
        total_y += sample.point.y
        total_z += sample.point.z

    count = len(samples)
    mean_point = Point(x=total_x / count, y=total_y / count, z=total_z / count)
    return PointStamped(header=samples[-1].header, point=mean_point)


class TargetQuery(Node):
    """订阅地图位置，并提供即时查询和可反馈、可取消的多帧观测任务。"""

    def __init__(self):
        """创建订阅、Service、Action 和用于检查取消、超时的定时器。"""
        super().__init__('target_query')
        self.declare_parameter('target_max_age_sec', 1.0)
        self.latest = None  # 最近收到的地图位置，用于 Service 查询。
        self.observation = None  # 当前 Action 任务；None 表示空闲。
        self.point_subscription = self.create_subscription(
            PointStamped, '/target/point_map', self.on_point, 10,
        )
        self.latest_service = self.create_service(GetTarget, '/target/get_latest', self.get_latest)
        # Action 的生命周期：检查请求 → 接受并启动 → 等待结果 → 成功/取消/超时。
        self.action = ActionServer(
            self, ObserveTarget, '/target/observe', self.execute_goal,
            goal_callback=self.check_request, handle_accepted_callback=self.start_goal,
            cancel_callback=self.allow_cancel,
        )
        # 独立于 ROS 回放时间，即使没有新观测或 /clock 暂停，也会检查取消和超时。
        self.deadline_timer = self.create_timer(
            0.05, self.check_deadline, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def get_latest(self, _request, response):
        """Service 回调：填写并返回 GetTarget 响应；请求为空，所以不读取 _request。"""
        if self.latest is None:
            response.message = 'No observation received'
            return response
        # 观测新鲜度必须使用 ROS 时间：回放时 now() 与消息都来自 bag 的时间线。
        observation_time = Time.from_msg(self.latest.header.stamp)
        age_sec = (self.get_clock().now() - observation_time).nanoseconds / 1e9
        max_age_sec = self.get_parameter('target_max_age_sec').value
        # 时间跳变可能让旧缓存看起来来自“未来”，负年龄也应当作无效。
        if 0.0 <= age_sec <= max_age_sec:
            response.found = True
            response.position = self.latest
            response.message = 'Fresh observation'
        else:
            response.message = 'No fresh observation'
        return response

    def check_request(self, request):
        """检查帧数、超时和忙碌状态，返回是否接受此 Action 请求。"""
        if self.observation is not None:
            return GoalResponse.REJECT
        if not 1 <= request.required_samples <= 3000:
            return GoalResponse.REJECT
        if not 0 < request.timeout_sec <= 300:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def start_goal(self, goal):
        """目标被接受后创建任务状态，并安排执行回调。"""
        self.observation = Observation(goal)
        goal.execute()

    async def execute_goal(self, _goal):
        """等待当前任务的 Future，完成后把 ObserveTarget.Result 返回给客户端。"""
        # async/await 是这里必要的异步等待：等待期间交还执行权，
        # 让订阅回调继续收集样本、定时器继续检查取消和超时。
        # 如果改成 while + sleep 阻塞等待，这些回调就可能无法运行。
        # 本节点同时只允许一个任务，所以这里读取的就是 start_goal 创建的状态。
        try:
            return await self.observation.result
        finally:
            # 结果交付后恢复空闲，后续请求才能再次通过 check_request。
            self.observation = None

    def allow_cancel(self, _goal):
        """允许取消请求；实际终止与结果返回由 check_deadline 完成。"""
        return CancelResponse.ACCEPT

    def check_deadline(self):
        """检查当前任务是否需要取消或超时结束，并填入对应的 Action 结果。"""
        observation = self.observation
        if observation is None or observation.result.done():
            return
        count = len(observation.samples)
        # 先更新 Action 状态，再填入 Future，唤醒 execute_goal 返回结果。
        # result.done() 防止同一任务被后续回调重复结束。
        if observation.goal.is_cancel_requested:
            observation.goal.canceled()
            observation.result.set_result(ObserveTarget.Result(samples_collected=count, message='Canceled'))
        elif monotonic() >= observation.deadline:
            observation.goal.abort()
            observation.result.set_result(ObserveTarget.Result(samples_collected=count, message='Timed out'))

    def on_point(self, point):
        """位置订阅回调：更新查询缓存；有活动任务时收集样本并发布反馈。"""
        self.latest = point
        # 先检查取消/超时，避免已到期的任务又因这一帧恰好凑齐样本而成功。
        self.check_deadline()
        observation = self.observation
        if observation is None or observation.result.done():
            return
        observation.samples.append(point)
        count = len(observation.samples)
        feedback = ObserveTarget.Feedback(samples_collected=count, latest_position=point)
        observation.goal.publish_feedback(feedback)
        if count == observation.goal.request.required_samples:
            mean_position = calculate_mean_position(observation.samples)
            observation.goal.succeed()
            observation.result.set_result(ObserveTarget.Result(
                samples_collected=count, mean_position=mean_position,
                message='Collected the requested observations',
            ))


def main():
    """启动查询节点；退出时先销毁 Action 服务，再释放节点和 ROS 上下文。"""
    rclpy.init()
    node = TargetQuery()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
