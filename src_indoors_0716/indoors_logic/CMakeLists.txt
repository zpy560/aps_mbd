cmake_minimum_required(VERSION 3.8)
project(indoors_logic)

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
find_package(byd_custom_msgs REQUIRED)
# 查找 Eigen3
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)  # 加入nav_msgs依赖
find_package(std_srvs REQUIRED)

# 添加 include 目录
# 添加 include 目录
include_directories(include)
include_directories(include/${PROJECT_NAME})


# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen3_INCLUDE_DIRS}
#   ${NLOPT_INCLUDE_DIR}
#   include
# )

# 查找 NLopt（传统方式）
find_path(NLOPT_INCLUDE_DIR nlopt.hpp PATH_SUFFIXES nlopt)
find_library(NLOPT_LIBRARY nlopt)
find_package(spdlog REQUIRED)
find_package(PkgConfig REQUIRED)

if(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_LIBRARY)
    message(FATAL_ERROR "NLopt library not found!")
endif()

# # ...existing code...

add_executable(control_node 
                src/control_node.cpp 
                src/control_node_init.cpp 
                src/control_node_utils.cpp 
                src/control_node_callback.cpp 
                src/control.cpp 
                src/bezier_model.cpp 
                src/bezier_three.cpp 
                include/Logger/Logger.cpp)

# 添加依赖
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  geometry_msgs
  byd_custom_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  std_srvs
)
target_link_libraries(control_node 
                      ${catkin_LIBRARIES} 
                      ${NLOPT_LIBRARY} 
                      Eigen3::Eigen
                      yaml-cpp
                      fmt::fmt
                      spdlog::spdlog)

install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})

add_executable(perception_test src/perception_client.cpp )
# 添加依赖
ament_target_dependencies(perception_test
  rclcpp
  std_msgs
  geometry_msgs
  byd_custom_msgs
)
install(TARGETS perception_test DESTINATION lib/${PROJECT_NAME})

install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})

add_executable(bizer_to_mcu_server_test src/bizer_to_mcu_server.cpp )
# 添加依赖
ament_target_dependencies(bizer_to_mcu_server_test
  rclcpp
  std_msgs
  geometry_msgs
  byd_custom_msgs
)
install(TARGETS bizer_to_mcu_server_test DESTINATION lib/${PROJECT_NAME})

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
