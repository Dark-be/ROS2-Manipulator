cmake_minimum_required(VERSION 3.10.2)
project(arm_controller)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
#find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
#find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
#find_package(rviz_visual_tools REQUIRED)
#find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
#find_package(rosbag REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  ~/moveit_ws/src/moveit2_tutorials/doc/interactivity/include
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  #tf2_ros
  moveit_core
  #rviz_visual_tools
  #moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  #moveit_servo
  #moveit_task_constructor_core
  std_msgs
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_executable(controller
    src/controller.cpp)
target_include_directories(controller
  PUBLIC include)
ament_target_dependencies(controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
