cmake_minimum_required(VERSION 3.8)
project(indoors_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 如果有自定义msg/srv/action，放在下面
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlRes.msg"        # 如果有自定义消息，按需替换/删除
  "srv/BizerToMcu.srv"        # 如果有自定义服务，按需替换/删除
  "srv/PerceptionToBizer.srv"        # 如果有自定义服务，按需替换/删除
  "srv/Status.srv"        # 如果有自定义服务，按需替换/删除
  "srv/Taskcs.srv"        # 如果有自定义服务，按需替换/删除
  "srv/ControlToTask.srv"        # 如果有自定义服务，按需替换/删除
  "msg/PathPoints.msg"        # 如果有自定义服务，按需替换/删除
  "msg/TaskType.msg"        # 如果有自定义服务，按需替换/删除
  "msg/Trajectory.msg"        # 如果有自定义服务，按需替换/删除
  "msg/MotionState.msg"        # 如果有自定义服务，按需替换/删除
  # "action/YourCustomAction.action"  # 如果有动作
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
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

ament_package()
