cmake_minimum_required(VERSION 3.8)
project(ros2_perception_demo)

find_package(ament_cmake REQUIRED)

# 将 Python 脚本安装为 ROS 可执行节点，并去掉 .py 后缀。
# 例如安装后可通过 ros2 run ros2_perception_demo mock_rgbd 启动。
foreach(node mock_rgbd localize_target target_query)
  install(PROGRAMS nodes/${node}.py DESTINATION lib/${PROJECT_NAME} RENAME ${node})
endforeach()
# 把启动文件和配置一起安装，供 ros2 launch 和包路径查询使用。
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
