"""启动感知流程：live 使用模拟相机，replay 使用外部 rosbag 播放器的输入。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """供 ros2 launch 调用，返回启动参数、节点及其启动条件。"""
    package = 'ros2_perception_demo'
    # 从已安装包中查找配置目录，不依赖运行命令时所在的目录。
    config_directory = Path(get_package_share_directory(package)) / 'config'
    # LaunchConfiguration 要等启动时才求值，不能用普通 Python if 判断其内容。
    # EqualsSubstitution 将 mode 与 replay 比较，供条件与参数共同使用。
    is_replay = EqualsSubstitution(LaunchConfiguration('mode'), 'replay')
    # 回放时节点跟随 /clock；实时模式使用系统时间。显式指定 bool 避免按字符串处理。
    clock_parameters = {'use_sim_time': ParameterValue(is_replay, value_type=bool)}
    latest_tf_parameter = ParameterValue(LaunchConfiguration('use_latest_tf'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='live', choices=['live', 'replay']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_latest_tf', default_value='false', choices=['true', 'false']),
        # 回放模式不启动模拟相机，图像和 TF 由用户在另一终端运行的 bag 播放器提供。
        Node(
            package=package, executable='mock_rgbd', output='screen',
            condition=UnlessCondition(is_replay),
        ),
        # 参数列表按顺序加载，后面的设置可覆盖前面的同名参数。
        Node(
            package=package, executable='localize_target', output='screen',
            parameters=[
                str(config_directory / 'demo.yaml'),
                clock_parameters,
                {'use_latest_tf': latest_tf_parameter},
            ],
        ),
        Node(
            package=package, executable='target_query', output='screen',
            parameters=[str(config_directory / 'demo.yaml'), clock_parameters],
        ),
        # 仅在 rviz:=true 时启动图形界面，命令行或无显示器环境也可以运行 Demo。
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', str(config_directory / 'demo.rviz')], parameters=[clock_parameters],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
