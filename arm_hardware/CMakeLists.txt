cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)


include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(PLATFORM "x86_64")

link_directories(lib/${PLATFORM})
set(dependencies sagittarius_sdk pthread boost_system boost_thread)

add_executable(sagittarius_example src/sagittarius_example.cpp)
target_link_libraries(sagittarius_example ${dependencies})
ament_target_dependencies(sagittarius_example rclcpp sensor_msgs rclcpp_action control_msgs)

ament_package()


#set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
install(TARGETS sagittarius_example DESTINATION lib/${PROJECT_NAME})
#install(DIRECTORY lib/x86_64/ DESTINATION /usr/lib)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
