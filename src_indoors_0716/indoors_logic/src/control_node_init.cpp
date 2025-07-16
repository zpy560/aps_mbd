#include "control_node.hpp"
#include "status.hpp"
#include <std_msgs/msg/detail/int32__struct.hpp>

// 如果在cpp文件中，可以加上
using namespace std::chrono_literals;

ControlNode::ControlNode() : Node("control_node") {
  initial();
  // line_arc_point_step_ = 0.050;   // 单位是m

  Task_msg_.task = byd_custom_msgs::msg::TaskType::UNKNOWN;
  // trajectory_sub_ =
  // this->create_subscription<byd_custom_msgs::msg::Trajectory>("/pub_trajectory",
  // 10, std::bind(&ControlNode::trajectory_callback, this,
  // std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_w_odom", 1,
      std::bind(&ControlNode::pose_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 1,
      std::bind(&ControlNode::odom_callback, this, std::placeholders::_1));
  obstacle_status_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/obstacle_status_int", 1,
      std::bind(&ControlNode::obstacle_status_callback, this,
                std::placeholders::_1));

  motion_state_sub_ =
      this->create_subscription<byd_custom_msgs::msg::MotionState>(
          "/motion_state", rclcpp::SensorDataQoS(),
          std::bind(&ControlNode::motion_state_callback, this,
                    std::placeholders::_1));

  control_pub_ = this->create_publisher<byd_custom_msgs::msg::ControlRes>(
      "/control_to_uart", 10);

  car_op_status_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("/car_op_status", 10);
  // 创建定时器，10ms周期
  timer_ = this->create_wall_timer(
      10ms, std::bind(&ControlNode::timerCallback, this));

  // 1. 创建 /remote_or_not 服务端
  remote_service_ = this->create_service<byd_custom_msgs::srv::Status>(
      "/remote_or_not",
      std::bind(&ControlNode::handle_remote_request, this,
                std::placeholders::_1, std::placeholders::_2));
  task_to_planning_service_ =
      this->create_service<byd_custom_msgs::srv::Taskcs>(
          "/task_to_planning",
          std::bind(&ControlNode::handle_task_to_planning_request, this,
                    std::placeholders::_1, std::placeholders::_2));
  remote_control_service_ =
      this->create_service<byd_custom_msgs::srv::Controller>(
          "remote_controller",
          std::bind(&ControlNode::handle_service_remote_control_request, this,
                    std::placeholders::_1, std::placeholders::_2));

  // 2. 创建 /task_feedback 服务客户端
  feedback_client_ = this->create_client<byd_custom_msgs::srv::ControlToTask>(
      "/task_feedback");
  feedback_perception_client_ =
      this->create_client<std_srvs::srv::SetBool>("/to_perception");

  service_ = this->create_service<PerceptionToBizer>(
      "/shelf_pose", std::bind(&ControlNode::PerceptionToBizer_callback, this,
                               std::placeholders::_1, std::placeholders::_2));

  // client_ = this->create_client<BizerToMcu>("bizer_to_mcu");

  RCLCPP_INFO(this->get_logger(), "ControlNode initialized.");
  LOG_INFO("ControlNode initialized. "); //{:.4f}",M_PI);
}

void ControlNode::initial() {
  // 初始化
  v_car = 0;
  w_car = 0;
  current_pose_to_odom_.pose.position.x = 0.0;
  current_pose_to_odom_.pose.position.y = 0.0;
  current_pose_to_odom_.pose.position.z = 0.0;
  current_pose_to_odom_.pose.orientation.w = 1.0;
  current_pose_to_odom_.pose.orientation.x = 0.0;
  current_pose_to_odom_.pose.orientation.y = 0.0;
  current_pose_to_odom_.pose.orientation.z = 0.0;
  Ld_ = 0.6;
  Ld_start_ = 0.3;
  pose_received_ = false;
  v_default_ = 0.1;
  endpath_v_default_ = 0.4;
  rec_task_flag_ = false;
  is_save_to_file_ = false; // 是否保存日志到文件
  is_start_init_ = false;
  has_line_stop_ = false; // 是否有直线停车点
  num_task_ = 0; // 任务编号
  rotation_stop_ = 0;
  rotation_stop_num_ = 20;
  car_op_status_.data = static_cast<int32_t>(CarOpStatus::DEFAULT);

  current_v_w_.first = 0.0;
  current_pose_.x = 0.0;
  current_pose_.y = 0.0;
  current_pose_.z = 0.0;
  current_xy_.x = 0.0;
  current_xy_.y = 0.0;
  current_xy_.speed = 0.0;
  line_arc_point_step_ = 0.050; // 单位是m
  line_expand_step_ = 1.100;    // 单位是m
  current_v_w_.first = 0.0;
  current_v_w_.second = 0.0;
  carmaxaccel_ = 2.0;
  carmaxaccel_second_ = 1.0;
  carmaxaccel_first_ = 1.5;
  carmax_autoaccel_ = 0.5;
  stop_distance_ = 0.005; // 单位是m
  // carmaxdecel_ = -10.0;
  carmaxwaccel_ = 300.0 / 180 * M_PI;
  carmaxwaccel_second_ = 150.0 / 180 * M_PI;
  carmaxwaccel_first_ = 200.0 / 180 * M_PI;
  carmaxw_startaccel_ = 100.0 / 180 * M_PI;
  carmaxw_ = 30.0 / 180 * M_PI;
  // carmaxwdecel_ = -10.0;
  isreach_endpoint_flag_ = false; // 到达终点标志位
  startstop_distance_ = 0.300;    // 起停距离，单位是m
  distance_goal_ = 1000.0;        // init 目标距离，单位是m
  is_start_rotation_ = false;
  is_stop_rotation_ = false;
  cycle_time_ = 0.01; // 控制周期，单位是秒
  is_rotation_ = false;
  is_rotation_yaw_ = 0.0;
  start_v_ = 0.3;
  remote_control_valid_ = false;
  neg_v_flag_ = false;
  obstacle_status_.data = 4;
  endpath_run_ = false;
  use_odom_ = false; // 是否使用odom数据
  out_end_traj_ = 0;
  end_out_flag_ = true;
  distance_startend_ = 0.0;
  
  first_dec_ = 0.25;
  second_dec_ = 0.6;
  error = 0.0; // 误差初始化
  omega = 0.0; // 角速度初始化
  start_roat_init_ = false; // 是否开始旋转
  start_line_init_ = false; // 是否开始直线
  calcu_w = 0.0;
  // 初始化header，赋当前时间戳
  current_motion_state_.header.stamp = rclcpp::Clock().now();
  current_motion_state_.header.frame_id = "base_link"; // 根据需要设定坐标系

  // 初始化其他浮点变量，示例赋0
  current_motion_state_.v_car = 0.0f;
  current_motion_state_.w_car = 0.0f;
  current_motion_state_.v_lift = 0.0f;
  current_motion_state_.lift_height = 0.0f;
  current_motion_state_.w_shelf = 0.0f;
  current_motion_state_.yaw_shelf = 0.0f;

  epsilon_ = 0.0001;        // 死区误差，单位是m
  epsilon_roat = 0.001;     // 死区误差，单位是rad
  lift_max_speed_ = 0.100;  // 最大提升速度，单位是m/s
  lift_max_accel_ = 0.50;   // 最大提升加速度，单位是/10ms
  lift_goal_ = 0.0;         // 升降目标高度，单位是mm
  rotate_goal_ = 0.0;       // 托盘旋转目标角度，单位是rad
  lift_speed_ = 0.0;        // 升降速度，单位是mm/s
  rotate_speed_ = 0.0;      // 托盘旋转速度，单位是rad/s
  rotate_max_speed_ = 0.10; // 最大旋转速度，单位是rad/s
  rotate_max_accel_ = 0.05; // 最大旋转加速度，单位是rad/s^2
  startstopdis_ = 0.0;

  // remote_controller_para_
  remote_under_direction_ = '0';
  remote_linear_v_ = 0.0;
  remote_angular_v_ = 0.0;
  remote_upper_direction_ = '0';
  remote_lift_v_ = 0.0;
  remote_rotate_v_ = 0.0;
  closest_index_ = 0;
  target_index_ = 0;
  // remote_linear_v_ = 0.0;
}
