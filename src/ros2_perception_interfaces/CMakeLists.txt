cmake_minimum_required(VERSION 3.8)
project(ros2_perception_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 从 .action 和 .srv 定义生成 Python 等语言可导入的 ROS 接口。
# geometry_msgs 用于位置字段，action_msgs 提供 Action 所需的基础消息。
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ObserveTarget.action"
  "srv/GetTarget.srv"
  DEPENDENCIES action_msgs geometry_msgs
)

# 让使用此包的节点能够获得生成接口所需的运行时依赖。
ament_export_dependencies(rosidl_default_runtime)
ament_package()
