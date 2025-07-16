#include "control_node.hpp"

// 如果在cpp文件中，可以加上
using namespace std::chrono_literals;

// /remote_or_not 服务处理回调
void ControlNode::handle_remote_request(const std::shared_ptr<byd_custom_msgs::srv::Status::Request> request, std::shared_ptr<byd_custom_msgs::srv::Status::Response> response)
{
    LOG_INFO("Received /remote_or_not request navtype={}, runtype={}", request->navtype, request->runtype);

    // 示例逻辑：如果navtype和runtype符合某条件，返回true
    if (request->navtype == 1 && request->runtype == 2)
    {
        response->flag = true;
        // control_to_task_msg_.auto_remote = 1; // 设置为远程模式
    }
    else
    {
        response->flag = false;
        // control_to_task_msg_.auto_remote = 0; // 设置为非远程模式
    }

    // 根据判断，调用反馈通知
    call_feedback_service(response->flag);
}
// /remote_or_not 服务处理回调 底盘遥控响应
void ControlNode::handle_service_remote_control_request(const std::shared_ptr<byd_custom_msgs::srv::Controller::Request> request, std::shared_ptr<byd_custom_msgs::srv::Controller::Response> response)
{
    // RCLCPP_INFO(this->get_logger(), "Received /remote_or_not request under_direction=%d upper_direction=%d", request->under_direction, request->upper_direction);
    // RCLCPP_ERROR(this->get_logger(), "request->angular_v=%lf ", request->angular_v);
    remote_control_valid_ = true;

    remote_under_direction_ = request->under_direction;
    remote_linear_v_ = request->linear_v;
    remote_angular_v_ = request->angular_v;
    remote_upper_direction_ = request->upper_direction;
    remote_lift_v_ = request->lift_v;
    remote_rotate_v_ = request->rotate_v;

    switch (remote_under_direction_)
    {
    case 10:
        remote_control_valid_ = false;
        remote_under_direction_ = '0';
        remote_linear_v_ = 0.0;
        remote_angular_v_ = 0.0;
        remote_upper_direction_ = '0';
        remote_lift_v_ = 0.0;
        remote_rotate_v_ = 0.0;
        w_car = 0.0;
        v_car = 0.0;
        break;
    case 0: // qian
        // remote_linear_v_ = -remote_linear_v_;
        remote_angular_v_ = 0.0;
        break;
    case 1: // hou
        remote_linear_v_ = -remote_linear_v_;
        remote_angular_v_ = 0.0;
        break;
    case 2: //
        remote_linear_v_ = 0.0;
        // remote_angular_v_ = 0.0;
        break;
    case 3:
        remote_angular_v_ = -remote_angular_v_;
        remote_linear_v_ = 0.0;
        break;
    case 5:
        remote_angular_v_ = -remote_angular_v_;
        break;
    case 6:
        remote_linear_v_ = -remote_linear_v_;
        break;
    case 7:
        remote_linear_v_ = -remote_linear_v_;
        remote_angular_v_ = -remote_angular_v_;
        break;
    default:
        LOG_WARN("remote_under_direction_={}", remote_under_direction_);
        break;
    }

    response->flag = true;
    LOG_INFO("remote_linear_v_remote_angular_v_={},{}", remote_linear_v_, remote_angular_v_);
}

void ControlNode::call_feedback_service(bool is_remote) // 监听是否远程模式，并反馈服务调用
{
    if (!feedback_client_->wait_for_service(std::chrono::milliseconds(10))) // 等待服务 10ms
    {
        RCLCPP_WARN(this->get_logger(), "/task_feedback service is not available");
        return;
    }
    auto request = std::make_shared<byd_custom_msgs::srv::ControlToTask::Request>();
    request->auto_remote = is_remote ? 1 : 0;

    // 异步调用反馈服务
    // auto result_future = feedback_client_->async_send_request(request);
    auto result_future = feedback_client_->async_send_request(request, [this](rclcpp::Client<byd_custom_msgs::srv::ControlToTask>::SharedFuture future)
                                                              {
                                                                  auto response = future.get();
                                                                  if (response->flag)
                                                                  {
                                                                    LOG_INFO("Feedback remote service returned success");
                                                                  }
                                                                  else
                                                                  {
                                                                    LOG_WARN("Feedback remote service returned failure");
                                                                  }
                                                            });
}

void ControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_odom_ = *msg;
    reflactor::Pose pose_w_odom;
    Eigen::Vector3d pos0(current_pose_to_odom_.pose.position.x, current_pose_to_odom_.pose.position.y, current_pose_to_odom_.pose.position.z);
    Eigen::Quaterniond q0(current_pose_to_odom_.pose.orientation.w, current_pose_to_odom_.pose.orientation.x, current_pose_to_odom_.pose.orientation.y, current_pose_to_odom_.pose.orientation.z);

    pose_w_odom = reflactor::Pose(pos0, q0);

    Eigen::Vector3d pos1(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
    Eigen::Quaterniond q1(current_odom_.pose.pose.orientation.w, current_odom_.pose.pose.orientation.x, current_odom_.pose.pose.orientation.y, current_odom_.pose.pose.orientation.z);
    reflactor::Pose odom = reflactor::Pose(pos1, q1);

    reflactor::Pose fusion_pose;
    fusion_pose.SetMatrix(pose_w_odom.matrix * odom.matrix);


    LOG_INFO("predictx: {},{}, {}, {}", (current_odom_.header.stamp.sec + current_odom_.header.stamp.nanosec*1e-9),fusion_pose.position.x(),fusion_pose.position.y(), fusion_pose.yaw);
    LOG_INFO("odom_callback,{},{},{}",current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);

    current_pose_.x = fusion_pose.position.x();
    current_pose_.y = fusion_pose.position.y();
    current_pose_.z = fusion_pose.yaw;
    current_xy_.x = current_pose_.x;
    current_xy_.y = current_pose_.y;
    current_xy_.speed = current_pose_.z;
    current_yaw_ = fusion_pose.yaw;
    // odom_received_ = true;
    // RCLCPP_WARN(this->get_logger(), "current_odom_ is time : %u,%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
    // RCLCPP_WARN(this->get_logger(), "current_odom_ is available");
}

void ControlNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (use_odom_)
    {return;}
    
    current_pose_to_odom_ = *msg;
    LOG_INFO("current_pose_to_odom_ is available");
    LOG_INFO("pose_callback,{},{},{}",current_pose_to_odom_.pose.position.x,current_pose_to_odom_.pose.position.y,current_pose_to_odom_.pose.position.z);

}

void ControlNode::motion_state_callback(const byd_custom_msgs::msg::MotionState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_motion_state_ = *msg;
    // RCLCPP_WARN(this->get_logger(), "current_motion_state_ is available");
    // RCLCPP_ERROR(this->get_logger(), "current_motion_state_ car_v car_w: %lf,%lf",current_motion_state_.v_car,current_motion_state_.w_car);
}

void ControlNode::obstacle_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    LOG_INFO("Received obstacle status: {}", msg->data);
    obstacle_status_.data = msg->data;
    // 这里可以根据需要处理障碍物状态
}
