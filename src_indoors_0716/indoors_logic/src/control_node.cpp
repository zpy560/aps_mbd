#include "control_node.hpp"
#include <cstdint>
#include <memory>
#include <std_msgs/msg/detail/int32__struct.hpp>

// 如果在cpp文件中，可以加上
using namespace std::chrono_literals;

void ControlNode::timerCallback()
{
    LOG_INFO("remote_control_valid_ : {},rec_task_flag_: {}", remote_control_valid_, rec_task_flag_);
    if (remote_control_valid_) // remote_controller
    {
        current_v_w_ = vwprocess(v_car, w_car, remote_linear_v_, remote_angular_v_, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
        v_car = current_v_w_.first;
        w_car = current_v_w_.second;
        control_msg_.v = current_v_w_.first; // 示例速度
        control_msg_.w = current_v_w_.second;
        control_msg_.v_lift = 0.0;
        control_msg_.w_rotation = 0.0;
        LOG_INFO("remote_linear_v_:{:.4f}, remote_angular_v_:{:.4f},current_motion_state_.v_car:{:.4f},current_motion_state_.w_car:{:.4f}",
                 remote_linear_v_, remote_angular_v_, current_motion_state_.v_car, current_motion_state_.w_car);
    }
    else if (rec_task_flag_)
    {
        if (obstacle_status_.data == 0)
        {
            control_msg_.v = 0.0; // 示例速度
            control_msg_.w = 0.0;
            control_msg_.v_lift = 0.0;
            control_msg_.w_rotation = 0.0;
        }
        else if (obstacle_status_.data == 1)
        {
            current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, 0.0, 0.0, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
            control_msg_.v = current_v_w_.first; // 示例速度
            control_msg_.w = 0.0;
            control_msg_.v_lift = 0.0;
            control_msg_.w_rotation = 0.0;
        }
        else
        {
            /* code */
            double cal_obs_v;
            double cal_obs_w;
            if (Task_msg_.task == byd_custom_msgs::msg::TaskType::ENDPATH)
            {
                if (is_rotation_)
                {
                    // 处理原地旋转
                    if (obstacle_status_.data == 2)
                    {
                        cal_obs_w = first_dec_ * carmaxwaccel_second_;
                    }
                    else if (obstacle_status_.data == 3)
                    {
                        cal_obs_w = second_dec_ * carmaxwaccel_second_;
                    }
                    else
                    {
                        cal_obs_w = carmaxwaccel_second_;
                    }
                    error = is_rotation_yaw_ - current_yaw_;
                    error = normalizeAngle(error);
                    omega = controlAngularVelocity(error, current_motion_state_.w_car,
                                                          carmaxw_, cal_obs_w, cycle_time_);
                    current_v_w_ = vwprocess(0.0, current_motion_state_.w_car, 0.0, omega,
                                             carmaxaccel_first_, cal_obs_w, cycle_time_);
                    LOG_INFO("current_motion_state_.w_car:{:.4f}, omega:{:.4f}", current_motion_state_.w_car, omega);
                    LOG_INFO("is_rotation_yaw_:{:.4f}, current_yaw_:{:.4f}, current_v_w_.second:{:.4f}",
                             is_rotation_yaw_, current_yaw_, current_v_w_.second);
                    control_msg_.v = 0.0; // 纯旋转 ，没有速度
                    control_msg_.w = current_v_w_.second;
                    control_msg_.v_lift = 0.0;
                    control_msg_.w_rotation = 0.0;
                    if (error < 0.01 && error > -0.01)
                    {

                        if (rotation_stop_ < rotation_stop_num_)
                        {
                            rotation_stop_ = rotation_stop_ + 1;
                        }
                        else
                        {
                            is_rotation_ = false;

                            // 新增：调用feedback_perception_client_发送布尔true
                            if (feedback_perception_client_ &&
                                feedback_perception_client_->service_is_ready())
                            {
                                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                                req->data = true;
                                feedback_perception_client_->async_send_request(
                                    req,
                                    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture
                                               future)
                                    {
                                        auto res = future.get();
                                        if (res->success)
                                        {
                                            LOG_INFO("feedback_perception_client_ 发送true成功");
                                        }
                                        else
                                        {
                                            LOG_WARN("feedback_perception_client_ 发送true失败");
                                        }
                                    });
                            }
                            percerption_start_time_ = std::chrono::steady_clock::now();
                        }
                    }
                }
                if (!is_rotation_ && !endpath_run_)
                {
                    control_msg_.v = 0.0; // 示例速度
                    control_msg_.w = 0.0;
                    control_msg_.v_lift = 0.0;
                    control_msg_.w_rotation = 0.0;
                }
                if (endpath_run_) //
                {
                    /* 此阶段为货架识别后的末端路径计算及发布控制消息 */
                    LOG_WARN("TaskType::ENDPATH . Received end key path.");
                    end_trajectory_.path = merged_;
                    end_trajectory_.task.task = Task_msg_.task;
                    // 计算曲线点
                    // 选取路径中第一个点作为预瞄点
                    closest_index_ = ternarySearchClosestIndex(current_xy_, end_trajectory_);
                    target_index_ = closest_index_ + Ld_ * 20;
                    if (target_index_ > end_trajectory_.path.size() - 1)
                    {
                        target_index_ = end_trajectory_.path.size() - 1;
                    }
                    
                    // carmaxaccel_first_;
                    LOG_INFO("closest_index-target_index_ : {},{}", closest_index_, target_index_);
                    LOG_INFO("end_trajectory_.path[closest_index-target_index_].x---v_default_: {:.4f},{:.4f},{:.4f}",
                             end_trajectory_.path[closest_index_].x, end_trajectory_.path[target_index_].x, v_default_);
                    auto lookahead_point_msg = end_trajectory_.path[target_index_];
                    Eigen::Vector2d v2d_current_point(current_pose_.x, current_pose_.y);
                    Eigen::Vector2d v2d_start_point(start_point_.x, start_point_.y);
                    Eigen::Vector2d v2d_end_point(end_point_.x, end_point_.y);
                    if (v_default_ < 1 || v_default_ < -1)
                    {
                        startstop_distance_ = v_default_ * v_default_ / 2 / carmaxaccel_second_;
                    }
                    else
                    {
                        startstop_distance_ = v_default_ * v_default_ / 2 / carmaxaccel_second_;
                    }
                    isreach_endpoint_flag_ = isProjectionOnSegment(
                        v2d_start_point, v2d_end_point, v2d_current_point);
                    distance_goal_ = std::sqrt(std::pow(current_pose_.x - end_point_.x, 2) +
                                               std::pow(current_pose_.y - end_point_.y, 2));
                    LOG_INFO("distance_goal_--startstop_distance_--isreach_endpoint_flag_ : {:.4f},{:.4f},{}",
                             distance_goal_, startstop_distance_, isreach_endpoint_flag_);

                    if (distance_goal_ < stop_distance_ || isreach_endpoint_flag_)
                    {
                        control_msg_.v = 0.0; // 示例速度
                        control_msg_.w = 0.0;
                        control_msg_.v_lift = 0.0;
                        control_msg_.w_rotation = 0.0;
                        endpath_run_ = false;
                    }
                    else if (distance_goal_ < startstop_distance_)
                    {
                        LOG_WARN("start_stop_flag_ ");
                        if (neg_v_flag_)
                        {
                            cal_obs_v = -0.05;
                        }
                        else
                        {
                            cal_obs_v = 0.05;
                        }
                        current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, cal_obs_v, 0.0, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
                        if (obstacle_status_.data == 2)
                        {
                            cal_obs_v = first_dec_ * v_default_;
                            if (abs(cal_obs_v) < abs(current_v_w_.first))
                            {
                                current_v_w_.first = cal_obs_v;
                            }
                        }
                        else if (obstacle_status_.data == 3)
                        {
                            cal_obs_v = second_dec_ * v_default_;
                            if (abs(cal_obs_v) < abs(current_v_w_.first))
                            {
                                current_v_w_.first = cal_obs_v;
                            }
                        }
                        else
                        {
                        }
                        LOG_INFO("cal_obs_v:{:.4f}, current_v_w_.first:{:.4f}", cal_obs_v, current_v_w_.first);
                        control_msg_.v = current_v_w_.first; // 示例速度
                        control_msg_.w = 0.0;
                        control_msg_.v_lift = 0.0;
                        control_msg_.w_rotation = 0.0;
                    }
                    else
                    {
                        // 计算转弯半径
                        double radius = ppc_.pure_pursuit_control(lookahead_point_msg,current_pose_, Ld_);
                        LOG_INFO("PPC control: Ld={},R={},car_pos=({},{},{} rad),target=({},{},{})", Ld_, radius, current_pose_.x, current_pose_.y,
                                 current_pose_.z, lookahead_point_msg.x, lookahead_point_msg.y, lookahead_point_msg.z);
                        if (obstacle_status_.data == 2)
                        {
                            cal_obs_w = first_dec_ * carmaxwaccel_second_;
                            cal_obs_v = first_dec_ * endpath_v_default_;
                        }
                        else if (obstacle_status_.data == 3)
                        {
                            cal_obs_w = second_dec_ * carmaxwaccel_second_;
                            cal_obs_v = second_dec_ * endpath_v_default_;
                        }
                        else
                        {
                            cal_obs_v = endpath_v_default_;
                            cal_obs_w = carmaxwaccel_second_;
                        }
                        calcu_w = cal_obs_v / radius;
                        current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, cal_obs_v, calcu_w, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
                        control_msg_.v = current_v_w_.first; // 示例速度
                        control_msg_.w = current_v_w_.second;
                        control_msg_.v_lift = 0.0;
                        control_msg_.w_rotation = 0.0;
                    }
                }
            }
            else if (Task_msg_.task == byd_custom_msgs::msg::TaskType::LIFT) // LIFT 升降模式
            {
                lift_speed_ = liftvprocess(current_motion_state_.lift_height, lift_goal_, lift_max_speed_, lift_max_accel_,
                                           cycle_time_, epsilon_, startstopdis_);
                LOG_INFO("lift_max_speed_,lift_speed_:{:.4f},{:.4f}", lift_max_speed_, lift_speed_);
                control_msg_.v = 0.0; // 示例速度
                control_msg_.w = 0.0;
                control_msg_.v_lift = lift_speed_;
                control_msg_.w_rotation = 0.0;
            }
            else if (Task_msg_.task == byd_custom_msgs::msg::TaskType::ROAT) // ROAT 旋转模式
            {
                LOG_INFO("TaskType::ROAT ");
                rotate_speed_ = roatvprocess(current_motion_state_.yaw_shelf, rotate_goal_, rotate_max_speed_,
                                             rotate_max_accel_, cycle_time_, epsilon_roat, startstopdis_);
                control_msg_.v = 0.0; // 示例速度
                control_msg_.w = 0.0;
                control_msg_.v_lift = 0.0;
                control_msg_.w_rotation = rotate_speed_;
            }
            else if (Task_msg_.task == byd_custom_msgs::msg::TaskType::NORMAL || Task_msg_.task == byd_custom_msgs::msg::TaskType::NORMALROAT) // NORMAL // 正常模式
            {
                LOG_INFO("TaskType NORMAL OR NORMALROAT");
                if (1 == is_start_init_)
                {
                    distance_goal_ = std::sqrt(std::pow(current_pose_.x - start_point_.x, 2) + std::pow(current_pose_.y - start_point_.y, 2));
                    LOG_INFO("distance_goal_:{},is_start_yaw_:{},is_rotation_yaw_:{},is_rotation_:{}",distance_goal_,is_start_yaw_,is_rotation_yaw_,is_rotation_);

                    cal_obs_w = carmaxw_startaccel_ * 0.5; // 计算障碍物角速度
                    if (is_rotation_)
                    {
                        error = is_start_yaw_ - current_yaw_;
                        error = normalizeAngle(error);
                        if (abs(error)<0.02)
                        {
                            control_msg_.w = 0.0;
                            if (rotation_stop_ < rotation_stop_num_)
                            {
                                rotation_stop_ = rotation_stop_ + 1;
                            }
                            else
                            {
                                is_rotation_ = false;
                                start_line_init_ = true;
                                rotation_stop_ = 0;
                            }
                        }
                        else
                        {
                            omega = controlAngularVelocity(error, current_motion_state_.w_car, carmaxw_*0.5, cal_obs_w, cycle_time_);
                            current_v_w_ = vwprocess(0.0, current_motion_state_.w_car, 0.0, omega, carmaxw_startaccel_, cal_obs_w, cycle_time_);
                            LOG_INFO("current_motion_state_.w_car:{:.4f}, omega:{:.4f}", current_motion_state_.w_car, omega);
                            LOG_INFO("is_start_yaw_:{:.4f}, current_yaw_:{:.4f}, current_v_w_.second:{:.4f},error:{}", is_start_yaw_, current_yaw_, current_v_w_.second,error);
                            control_msg_.w = current_v_w_.second;
                        }
                        control_msg_.v = 0.0; // 纯旋转 ，没有速度
                        
                    }
                    else if (start_line_init_)
                    {
                        Eigen::Vector2d v_current_point(current_pose_.x, current_pose_.y);
                        Eigen::Vector2d v_start_point(is_start_end_.x, is_start_end_.y);
                        Eigen::Vector2d v_end_point(start_point_.x, start_point_.y);
                        end_trajectory_.path = startline1_;
                        end_trajectory_.task.task = Task_msg_.task;
                        isreach_endpoint_flag_ = isProjectionOnSegment(v_start_point, v_end_point, v_current_point);
                        LOG_INFO("start_line_init_--current_pose_ x:{:.4f}, y:{:.4f}, isreach_endpoint_flag_={:.4f}", current_pose_.x, current_pose_.y, isreach_endpoint_flag_);
                        closest_index_ = ternarySearchClosestIndex(current_xy_, end_trajectory_);
                        target_index_ = closest_index_ + Ld_ * 20;
                        if (target_index_ > end_trajectory_.path.size() - 1)
                        {
                            target_index_ = end_trajectory_.path.size() - 1;
                        }
                        LOG_INFO("closest_index-target_index_ : {},{}", closest_index_, target_index_);
                        LOG_INFO("end_trajectory_.path[closest_index-target_index_].x:{:.4f},{:.4f}", end_trajectory_.path[closest_index_].x,end_trajectory_.path[target_index_].x);
                        auto lookahead_point_msg_init = end_trajectory_.path[target_index_];

                        if ((distance_goal_ < 5*stop_distance_ || isreach_endpoint_flag_))
                        {
                            if (rotation_stop_ < rotation_stop_num_)
                            {
                                rotation_stop_ = rotation_stop_ + 1;
                            }
                            else
                            {
                                start_line_init_ = false;
                                start_roat_init_ = true;
                                rotation_stop_ = 0;
                                
                            }
                            control_msg_.v = 0.0; // 示例速度
                            control_msg_.w = 0.0;
                        }
                        else if (distance_goal_ < 0.2)
                        {
                            current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, 0.05, 0.0, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
                            control_msg_.v = current_v_w_.first; // 示例速度
                            control_msg_.w = current_v_w_.second;
                        }
                        else
                        {
                            double radius_start = ppc_.pure_pursuit_control(lookahead_point_msg_init, current_pose_, Ld_start_);
                            LOG_INFO("PPC control: Ld_start_={},R={},car_pos=({},{},{} rad),target=({},{},{})", Ld_start_, radius_start, current_pose_.x, current_pose_.y,
                                    lookahead_point_msg_init.z, lookahead_point_msg_init.x, lookahead_point_msg_init.y, lookahead_point_msg_init.z);
                            calcu_w = 0.3 / radius_start;
                            current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, 0.3, calcu_w, carmaxaccel_first_, cal_obs_w, cycle_time_);
                            control_msg_.v = current_v_w_.first; // 示例速度
                            control_msg_.w = current_v_w_.second;
                        }
                    }
                    else if (start_roat_init_)
                    {
                        error = is_rotation_yaw_ - current_yaw_;
                        error = normalizeAngle(error);
                        
                        if (abs(error)<0.02)
                        {
                            control_msg_.w = 0.0;
                            if (rotation_stop_ < rotation_stop_num_)
                            {
                                rotation_stop_ = rotation_stop_ + 1;
                            }
                            else
                            {
                                is_rotation_ = false;
                                start_line_init_ = false;
                                is_start_init_ = 0;
                                start_roat_init_ = false;
                                rotation_stop_ = 0;
                            }
                        }
                        else
                        {
                            omega = controlAngularVelocity(error, current_motion_state_.w_car, carmaxw_*0.5, cal_obs_w, cycle_time_);
                            current_v_w_ = vwprocess(0.0, current_motion_state_.w_car, 0.0, omega, carmaxw_startaccel_, cal_obs_w, cycle_time_);
                            LOG_INFO("current_motion_state_.w_car:{:.4f}, omega:{:.4f}", current_motion_state_.w_car, omega);
                            LOG_INFO("is_rotation_yaw_:{:.4f}, current_yaw_:{:.4f}, current_v_w_.second:{:.4f},error:{}", is_rotation_yaw_, current_yaw_, current_v_w_.second,error);
                            control_msg_.w = current_v_w_.second;
                        }
                        control_msg_.v = 0.0; // 纯旋转 ，没有速度
                        
                    }
                    else
                    {
                        LOG_INFO("start_init_,is_rotation_:{},start_line_init_{},start_roat_init_{}",is_rotation_,start_line_init_,start_roat_init_);
                    }
                    control_msg_.v_lift = 0.0;
                    control_msg_.w_rotation = 0.0;
                    LOG_INFO("rotation_stop_={},start_line_init_={}", rotation_stop_,start_line_init_);
                }
                else if (2 == is_start_init_)
                {
                    // 处理原地旋转
                    if (obstacle_status_.data == 2)
                    {
                        cal_obs_w = first_dec_ * carmaxwaccel_second_;
                    }
                    else if (obstacle_status_.data == 3)
                    {
                        cal_obs_w = second_dec_ * carmaxwaccel_second_;
                    }
                    else
                    {
                        cal_obs_w = carmaxw_startaccel_;
                    }
                    error = is_rotation_yaw_ - current_yaw_;
                    error = normalizeAngle(error);
                    omega = controlAngularVelocity(error, current_motion_state_.w_car, carmaxw_, cal_obs_w, cycle_time_);
                    current_v_w_ = vwprocess(0.0, current_motion_state_.w_car, 0.0, omega, carmaxaccel_first_, cal_obs_w, cycle_time_);
                    LOG_INFO("current_motion_state_.w_car:{:.4f}, omega:{:.4f}", current_motion_state_.w_car, omega);
                    LOG_INFO("is_rotation_yaw_:{:.4f}, current_yaw_:{:.4f}, current_v_w_.second:{:.4f},error:{}", is_rotation_yaw_, current_yaw_, current_v_w_.second,error);
                    control_msg_.v = 0.0; // 纯旋转 ，没有速度
                    control_msg_.w = current_v_w_.second;
                    control_msg_.v_lift = 0.0;

                    if (Task_msg_.task == byd_custom_msgs::msg::TaskType::NORMALROAT)
                    {
                        car_op_status_.data = 3;
                        control_msg_.w_rotation = -control_msg_.w;
                    }
                    else
                    {
                        control_msg_.w_rotation = 0.0;
                    }

                    if (abs(error)<0.02)
                    {
                        control_msg_.w = 0.0;
                        if (rotation_stop_ < rotation_stop_num_)
                        {
                            rotation_stop_ = rotation_stop_ + 1;
                        }
                        else
                        {
                            is_start_init_ = 0;
                            car_op_status_.data = 6;
                            rotation_stop_ = 0;
                        }
                    }
                }
                else
                {
                    double carmaxaccel_stop;
                    if (abs(v_default_) < 1)
                    {
                        startstop_distance_ = v_default_ * v_default_ * 1.2;
                        carmaxaccel_stop = carmaxaccel_first_;
                    }
                    else if (abs(v_default_) < 1.21)
                    {
                        startstop_distance_ = v_default_ * v_default_ / 2  * 1.8;
                        carmaxaccel_stop = carmaxaccel_first_;
                    }
                    else
                    {
                        startstop_distance_ = v_default_ * v_default_ / 2 * 1.4 ;
                        carmaxaccel_stop = carmaxaccel_;
                    }
                    if (has_line_stop_)
                    {
                        Eigen::Vector2d v2d_current_point(current_pose_.x, current_pose_.y);
                        Eigen::Vector2d v2d_start_point(start_point_.x, start_point_.y);
                        Eigen::Vector2d v2d_end_point(end_point_.x, end_point_.y);
                        end_trajectory_.path = line1_;
                        end_trajectory_.task.task = Task_msg_.task;
                        isreach_endpoint_flag_ = isProjectionOnSegment(v2d_start_point, v2d_end_point, v2d_current_point);
                        LOG_INFO("current_pose_ x:{:.4f}, y:{:.4f}, isreach_endpoint_flag_={:.4f}", current_pose_.x, current_pose_.y, isreach_endpoint_flag_);
                    }
                    else
                    {
                        end_trajectory_.path = merged_;
                        end_trajectory_.task.task = Task_msg_.task;
                    }
                    LOG_INFO("current_pose_ x:{:.4f}, y:{:.4f}, yaw:{:.4f}, v_default_={:.4f}", current_pose_.x, current_pose_.y, current_pose_.z * 180.0 / M_PI, v_default_);
                    closest_index_ = ternarySearchClosestIndex(current_xy_, end_trajectory_);
                    target_index_ = closest_index_ + Ld_ * 20;
                    if (target_index_ > end_trajectory_.path.size() - 1)
                    {
                        target_index_ = end_trajectory_.path.size() - 1;
                    }
                    LOG_INFO("closest_index-target_index_ : {},{}", closest_index_, target_index_);
                    LOG_INFO("end_trajectory_.path[closest_index-target_index_].x:{:.4f},{:.4f}", end_trajectory_.path[closest_index_].x,end_trajectory_.path[target_index_].x);
                    auto lookahead_point_msg = end_trajectory_.path[target_index_];
                    distance_goal_ = std::sqrt(std::pow(current_pose_.x - end_point_.x, 2) + std::pow(current_pose_.y - end_point_.y, 2));
                    LOG_INFO("distance_goal_-isreach_endpoint_flag_ : {:.4f},{},startstop_distance_:{}", distance_goal_, isreach_endpoint_flag_,startstop_distance_);

                    if ((distance_goal_ < stop_distance_ || isreach_endpoint_flag_) && has_line_stop_)
                    {
                        control_msg_.v = 0.0; // 示例速度
                        control_msg_.w = 0.0;
                        control_msg_.v_lift = 0.0;
                        control_msg_.w_rotation = 0.0;
                        rec_task_flag_ = false;
                        if (rotation_stop_ < rotation_stop_num_)
                        {
                            rotation_stop_ = rotation_stop_ + 1;
                        }
                        else
                        {
                            rotation_stop_ = 0;
                        }
                    }
                    else if (distance_goal_ < startstop_distance_ && has_line_stop_)
                    {
                        LOG_INFO("start_stop_flag_ : {}", start_stop_flag_);
                        if (neg_v_flag_)
                        {
                            cal_obs_v = -0.05;
                        }
                        else
                        {
                            cal_obs_v = 0.05;
                        }
                        // current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, cal_obs_v, 0.0, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
                        current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, cal_obs_v, 0.0, carmaxaccel_stop, carmaxwaccel_second_, cycle_time_);
                        LOG_INFO("cal_obs_v:{:.4f}, current_v_w_.first:{:.4f}", cal_obs_v, current_v_w_.first);
                        control_msg_.v = current_v_w_.first; // 示例速度
                        control_msg_.w = 0.0;
                        control_msg_.v_lift = 0.0;
                        control_msg_.w_rotation = 0.0;
                    }
                    else
                    {
                        // 计算转弯半径
                        double radius = ppc_.pure_pursuit_control(lookahead_point_msg, current_pose_, Ld_);
                        if (obstacle_status_.data == 2)
                        {
                            cal_obs_w = first_dec_ * carmaxwaccel_second_;
                            cal_obs_v = first_dec_ * v_default_;
                        }
                        else if (obstacle_status_.data == 3)
                        {
                            cal_obs_w = second_dec_ * carmaxwaccel_second_;
                            cal_obs_v = second_dec_ * v_default_;
                        }
                        else
                        {
                            cal_obs_v = v_default_;
                            cal_obs_w = carmaxwaccel_second_;
                        }
                        if (end_out_flag_ && distance_goal_>0.8*distance_startend_)
                        {
                            cal_obs_v = v_default_*0.3;
                        }
                        calcu_w = cal_obs_v / radius;
                        current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, cal_obs_v, calcu_w, carmaxaccel_first_, cal_obs_w, cycle_time_);
                        control_msg_.v = current_v_w_.first; // 示例速度
                        control_msg_.w = current_v_w_.second;
                        control_msg_.v_lift = 0.0;
                        control_msg_.w_rotation = 0.0;
                        LOG_INFO("PPC control: Ld={},R={},car_pos=({},{},{} rad),target=({},{},{}),cal_obs_v = {}", Ld_, radius, current_pose_.x, current_pose_.y,
                                 current_pose_.z, lookahead_point_msg.x, lookahead_point_msg.y, lookahead_point_msg.z,cal_obs_v);
                    }
                }
                LOG_INFO("rotation_stop_:{}",rotation_stop_);
            }
            else
            {
                current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, 0.0, 0.0, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
                control_msg_.v = current_v_w_.first; // 示例速度
                control_msg_.w = current_v_w_.second;
                control_msg_.v_lift = 0.0;
                control_msg_.w_rotation = 0.0;
            }
        }
    }
    else
    {
        current_v_w_ = vwprocess(current_motion_state_.v_car, current_motion_state_.w_car, 0.0, 0.0, carmaxaccel_first_, carmaxwaccel_second_, cycle_time_);
        control_msg_.v = current_v_w_.first; // 示例速度
        control_msg_.w = current_v_w_.second;
        control_msg_.v_lift = 0.0;
        control_msg_.w_rotation = 0.0;
    }
    LOG_INFO("current_motion_state_.v_car:{:.4f},ControlRes:calcu_v:{:.4f},w:{:.4f}, yaw:{:.4f} deg", current_motion_state_.v_car, control_msg_.v, control_msg_.w,current_pose_.z * 180.0 / M_PI);
    control_pub_->publish(control_msg_);
    car_op_status_pub_->publish(car_op_status_);
}

void ControlNode::PerceptionToBizer_callback(const std::shared_ptr<PerceptionToBizer::Request> req,
                                             std::shared_ptr<PerceptionToBizer::Response> res)
{
    percerption_end_time_ = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(percerption_end_time_ - percerption_start_time_).count();
    LOG_INFO("percerption_time_ 耗时: {:.4f} ms", elapsed_ms);
    Eigen::Vector3d startPos(0, 0, 0);
    Eigen::Vector3d endPos_last(1000, 100, 0);
    Eigen::Vector3d endPos(1000, 100, 0);
    Eigen::Vector3d endPos_mid(1000, 100, 0);
    geometry_msgs::msg::Point point;
    BezierModel model(5, 1, 0.000004, 0.000002, 0.000001);
    tf2::Quaternion quat(req->send_points[0].pose.orientation.x,
                         req->send_points[0].pose.orientation.y,
                         req->send_points[0].pose.orientation.z,
                         req->send_points[0].pose.orientation.w);
    tf2::Quaternion quat1(req->send_points[1].pose.orientation.x,
                          req->send_points[1].pose.orientation.y,
                          req->send_points[1].pose.orientation.z,
                          req->send_points[1].pose.orientation.w);
    res->flag = true;
    double roll, pitch, yaw;
    startPos(0) = current_pose_.x * 1000;
    startPos(1) = current_pose_.y * 1000;
    startPos(2) = current_pose_.z;

    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    endPos_mid(0) = req->send_points[0].pose.position.x * 1000;
    endPos_mid(1) = req->send_points[0].pose.position.y * 1000;
    endPos_mid(2) = yaw;

    tf2::Matrix3x3(quat1).getRPY(roll, pitch, yaw);
    endPos_last(0) = req->send_points[1].pose.position.x * 1000;
    endPos_last(1) = req->send_points[1].pose.position.y * 1000;
    endPos_last(2) = yaw;
    auto request = std::make_shared<byd_custom_msgs::srv::ControlToTask::Request>();
    geometry_msgs::msg::Point point_msg;
    point_msg.x = endPos_last(0) / 1000;
    point_msg.y = endPos_last(1) / 1000;
    point_msg.z = endPos_last(2);
    request->auto_remote = 10;
    request->end_point = point_msg;
    auto result_future = feedback_client_->async_send_request(request, [this](rclcpp::Client<byd_custom_msgs::srv::ControlToTask>::SharedFuture future)
                                                              {
            auto response = future.get();
            if (response->flag)
            {LOG_INFO("response->flag: {}", response->flag);}
            else
            {LOG_INFO("response->flag: {}", response->flag);} });
    use_odom_ = true;
    out_end_traj_ = 2;
    LOG_INFO("PerceptionToBizer_callback Received ,current_pose_: x {:.4f}, y {:.4f}, z {:.4f}", startPos(0), startPos(1), startPos(2));
    LOG_INFO("PerceptionToBizer_callback Received ,req->send_points[0]: x {:.4f}, y {:.4f}, z {:.4f}", endPos_mid(0), endPos_mid(1), endPos_mid(2));
    LOG_INFO("PerceptionToBizer_callback Received ,req->send_points[1]: x {:.4f}, y {:.4f}, z {:.4f}", endPos_last(0), endPos_last(1), endPos_last(2));
    double dlength = 300;
    Eigen::Vector3d direction = -endPos_last + endPos_mid;
    direction.normalize();
    endPos = endPos_mid + direction * dlength;
    LOG_INFO("PerceptionToBizer_callback 计算得到的endPos: x {:.4f}, y {:.4f}, z {:.4f}", endPos(0), endPos(1), endPos(2));
    percerption_end_time_ = std::chrono::steady_clock::now();
    model.setStartPoint(startPos);
    model.setEndPoint(endPos);
    model.initializeControlPoints();
    model.setCurvatueRange(5, -5);
    model.setSampleTimes(1000);
    model.setKesai(15, -15);
    model.optimizeControlPoints();
    calendpath_start_time_ = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration<double, std::milli>(calendpath_start_time_ - percerption_end_time_).count();
    LOG_INFO("calendpath_time_ 耗时: {:.4f} ms", elapsed_ms);

    if (model.controlPoints.rows() != 6)
    {
        LOG_INFO("Control points size is not 6. model.controlPoints.size():{},PerceptionToBizer_callback failed.", model.controlPoints.rows());
        res->flag = false;
        return;
    }
    LOG_INFO("Request_start: x={:.4f}, y={:.4f}, z={:.4f}", startPos(0), startPos(1), startPos(2));
    LOG_INFO("Request_end: x={:.4f}, y={:.4f}, z={:.4f}", endPos(0), endPos(1), endPos(2));

    // 假设contr_points大小不少于8
    for (int i = 0; i < model.controlPoints.rows(); ++i)
    {
        point.x = model.controlPoints(i, 0);
        point.y = model.controlPoints(i, 1);
        point.z = 0.0;
        end_pose_.push_back(point);
        LOG_INFO("response: id={:.4f}, x={:.4f}, y={:.4f}, z={:.4f}", i, point.x, point.y, point.z);
    }

    if (model.controlPoints.rows() == 6)
    {
        line1_.clear();
        arc1_.clear();
        ctrl_pts_.clear();
        merged_.clear();
        line1_point3d_.clear();
        arc1_point3d_.clear();
        is_rotation_ = false;
        arc_speed_ = 0.4; // 默认设置圆弧速度
        for (long int i = 0; i < model.controlPoints.rows(); i++)
        {
            start_point_.x = model.controlPoints(i, 0) / 1000;
            start_point_.y = model.controlPoints(i, 1) / 1000;
            start_point_.speed = 0.0;
            ctrl_pts_.push_back(start_point_);
        }
        arc1_point3d_ = generateUniformBezier5Curve(ctrl_pts_, line_arc_point_step_);
        arc1_ = update_trajectory(arc1_point3d_, arc_speed_);
        start_point_.x = endPos(0) / 1000;
        start_point_.y = endPos(1) / 1000;
        end_point_.x = endPos_last(0) / 1000;
        end_point_.y = endPos_last(1) / 1000;
        line_speed_ = arc_speed_;
        extend_end_point_ = extendEndPoint(start_point_, end_point_, line_expand_step_); // 延长终点固定距离
        line1_point3d_ = generateFixedStepPoints(start_point_, extend_end_point_, line_arc_point_step_);
        line1_ = update_trajectory(line1_point3d_, line_speed_);
        merged_.reserve(line1_.size() + arc1_.size());             // 预分配内存，提高效率
        merged_.insert(merged_.end(), arc1_.begin(), arc1_.end()); // 先圆弧后直线
        merged_.insert(merged_.end(), line1_.begin(), line1_.end());
    }
    LOG_INFO("PerceptionToBizer_callback 任务完成状态: {}", res->flag);
    Task_msg_.task = byd_custom_msgs::msg::TaskType::ENDPATH;
    endpath_run_ = true;
}

void ControlNode::handle_task_to_planning_request(const std::shared_ptr<byd_custom_msgs::srv::Taskcs::Request> request,
                                                  std::shared_ptr<byd_custom_msgs::srv::Taskcs::Response> response)
{
    LOG_INFO("Received /task_to_planning request task_id= {}", request->task.task);
    Task_msg_.task = request->task.task;
    response->flag = true;
    use_odom_ = false;
    rec_task_flag_ = true;
    rotation_stop_ = 0;
    num_task_ = num_task_ + 1;
    car_op_status_.data = static_cast<int32_t>(CarOpStatus::NAV);
    if (current_xy_.x == 0.0 && current_xy_.y == 0.0 && current_xy_.speed == 0.0)
    {
        LOG_INFO("current_pose_ is zero, waiting for odom data.");
        response->flag = false;
        return;
    }
    // is_start_init_ = checkisstart_init();
    if (request->task.task == byd_custom_msgs::msg::TaskType::LIFT)
    {
        lift_goal_ = request->path.front().x;
        startstopdis_ = lift_max_speed_ * lift_max_speed_ / (2 * lift_max_accel_);
        LOG_INFO("lift_goal_ : {:.4f},request->task.task: {}", lift_goal_, request->task.task);
    }
    else if (request->task.task == byd_custom_msgs::msg::TaskType::ROAT)
    {
        rotate_goal_ = request->path.front().x;
        startstopdis_ = rotate_max_speed_ * rotate_max_speed_ / (2 * rotate_max_accel_);
        LOG_INFO("rotate_goal_ : {:.4f},request->task.task: {}", rotate_goal_, request->task.task);
    }
    else if (request->task.task == byd_custom_msgs::msg::TaskType::NORMAL || Task_msg_.task == byd_custom_msgs::msg::TaskType::NORMALROAT)
    {

        if (Task_msg_.task == byd_custom_msgs::msg::TaskType::NORMALROAT)
        {
            car_op_status_.data = 3;
        }
        if (2 == out_end_traj_)
        {
            end_out_flag_ = true;
        }
        else
        {
            out_end_traj_ = 1;
        }

        line_speed_ = request->path[0].speed;
        v_default_ = line_speed_;
        start_point_.x = request->path[0].x;
        start_point_.y = request->path[0].y;
        end_point_.x = request->path[1].x;
        end_point_.y = request->path[1].y;
        distance_startend_ = std::sqrt(std::pow(start_point_.x - end_point_.x, 2) + std::pow(start_point_.y - end_point_.y, 2));
        LOG_INFO("request->task.task: {},start_point_ x={:.4f}, y={:.4f}, end_point_x={:.4f}, end_point_y={:.4f},v_default_={}",
                 request->task.task, start_point_.x, start_point_.y, end_point_.x, end_point_.y,v_default_);
        is_start_end_.x = current_xy_.x;
        is_start_end_.y = current_xy_.y;
        if (v_default_ < 0.0)
        {
            current_xy_.speed = current_xy_.speed + M_PI;
            current_xy_.speed = normalizeAngle(current_xy_.speed);
            is_start_init_ = checkisstart_init(start_point_, end_point_, current_xy_);
            neg_v_flag_ = true;
            if (1 == is_start_init_)
            {
                is_start_yaw_ = atan2((-start_point_.y + current_xy_.y), (-start_point_.x + current_xy_.x)); // calrotationyaw(start_point_,end_point_,current_xy_);
                is_rotation_yaw_ = atan2((-start_point_.y + end_point_.y), (-start_point_.x + end_point_.x));
                is_rotation_ = true;
                extend_end_point_ = extendEndPoint(is_start_end_,start_point_, line_expand_step_); // 延长终点固定距离
                startline_point3d_ = generateFixedStepPoints(is_start_end_, extend_end_point_, line_arc_point_step_);
                startline1_ = update_trajectory(startline_point3d_, -0.3);
            }

            else if (2 == is_start_init_)
            {
                is_rotation_yaw_ = atan2((start_point_.y - end_point_.y), (start_point_.x - end_point_.x)); // calrotationyaw(start_point_,end_point_,current_xy_);
            }
        }
        else
        {
            current_xy_.speed = normalizeAngle(current_xy_.speed);
            is_start_init_ = checkisstart_init(start_point_, end_point_, current_xy_);
            neg_v_flag_ = false;
            if (1 == is_start_init_)
            {
                is_start_yaw_ = atan2((start_point_.y - current_xy_.y), (start_point_.x - current_xy_.x)); // calrotationyaw(start_point_,end_point_,current_xy_);
                is_rotation_yaw_ = atan2((end_point_.y - start_point_.y), (end_point_.x - start_point_.x));
                is_rotation_ = true;
                extend_end_point_ = extendEndPoint(is_start_end_,start_point_, line_expand_step_); // 延长终点固定距离
                startline_point3d_ = generateFixedStepPoints(is_start_end_, extend_end_point_, line_arc_point_step_);
                startline1_ = update_trajectory(startline_point3d_, 0.3);
            }
            if (2 == is_start_init_)
            {
                is_rotation_yaw_ = atan2((end_point_.y - start_point_.y), (end_point_.x - start_point_.x)); // calrotationyaw(start_point_,end_point_,current_xy_);
            }
        }
        LOG_INFO("is_start_init_:{},is_rotation_yaw_:{},request->line_flag:{}",is_start_init_,is_rotation_yaw_,request->line_flag);

        if (request->line_flag == 1) // 双直线
        {
            line1_.clear();
            line2_.clear();
            merged_.clear();
            line1_point3d_.clear();
            line2_point3d_.clear();
            if (request->path.size() == 4)
            {
                start_point_.x = request->path[0].x;
                start_point_.y = request->path[0].y;
                end_point_.x = request->path[1].x;
                end_point_.y = request->path[1].y;
                line1_point3d_ = generateFixedStepPoints(start_point_, end_point_, line_arc_point_step_);
                line1_ = update_trajectory(line1_point3d_, line_speed_);
                start_point_.x = request->path[2].x;
                start_point_.y = request->path[2].y;
                end_point_.x = request->path[3].x;
                end_point_.y = request->path[3].y;
                line_speed_ = request->path[2].speed;
                line2_point3d_ = generateFixedStepPoints(start_point_, end_point_, line_arc_point_step_);
                line2_ = update_trajectory(line2_point3d_, line_speed_);
                merged_.reserve(line1_.size() + line2_.size());              // 预分配内存，提高效率
                merged_.insert(merged_.end(), line1_.begin(), line1_.end()); // // 先直线后圆弧
                merged_.insert(merged_.end(), line2_.begin(), line2_.end());
            }
            else
            {
                response->flag = false;
                LOG_INFO("control_points.size is not 4. is path.size()={}", request->path.size());
            }
            // response->flag = true;
        }
        else if (request->line_flag == 2) // 直线+圆弧
        {
            line1_.clear();
            arc1_.clear();
            ctrl_pts_.clear();
            merged_.clear();
            if (request->path.size() == 6)
            {
                start_point_.x = request->path[0].x;
                start_point_.y = request->path[0].y;
                end_point_.x = request->path[1].x;
                end_point_.y = request->path[1].y;
                line1_point3d_ = generateFixedStepPoints(start_point_, end_point_, line_arc_point_step_);
                line1_ = update_trajectory(line1_point3d_, line_speed_);
                arc_speed_ = request->path[2].speed;
                for (size_t i = 2; i < request->path.size(); i++)
                {
                    start_point_.x = request->path[i].x;
                    start_point_.y = request->path[i].y;
                    start_point_.speed = 0.0;
                    ctrl_pts_.push_back(start_point_);
                }
                arc1_point3d_ = generateBezierUniformPoints(ctrl_pts_, line_arc_point_step_);
                arc1_ = update_trajectory(arc1_point3d_, arc_speed_);
                merged_.reserve(line1_.size() + arc1_.size());               // 预分配内存，提高效率
                merged_.insert(merged_.end(), line1_.begin(), line1_.end()); // // 先直线后圆弧
                merged_.insert(merged_.end(), arc1_.begin(), arc1_.end());
            }
            else
            {
                response->flag = false;
                LOG_INFO("control_points.size is not 6. is path.size()={}", request->path.size());
            }
        }
        else if (request->line_flag == 3) // 圆弧+直线
        {
            line1_.clear();
            arc1_.clear();
            ctrl_pts_.clear();
            merged_.clear();
            if (request->path.size() == 6)
            {
                for (size_t i = 0; i < request->path.size() - 2; i++)
                {
                    start_point_.x = request->path[i].x;
                    start_point_.y = request->path[i].y;
                    start_point_.speed = 0.0;
                    ctrl_pts_.push_back(start_point_);
                }
                arc1_point3d_ = generateBezierUniformPoints(ctrl_pts_, line_arc_point_step_);
                arc1_ = update_trajectory(arc1_point3d_, line_speed_);
                start_point_.x = request->path[4].x;
                start_point_.y = request->path[4].y;
                end_point_.x = request->path[5].x;
                end_point_.y = request->path[5].y;
                line_speed_ = request->path[4].speed;
                line1_point3d_ = generateFixedStepPoints(start_point_, end_point_, line_arc_point_step_);
                line1_ = update_trajectory(line1_point3d_, line_speed_);
                merged_.reserve(line1_.size() + arc1_.size());             // 预分配内存，提高效率
                merged_.insert(merged_.end(), arc1_.begin(), arc1_.end()); // 先圆弧后直线
                merged_.insert(merged_.end(), line1_.begin(), line1_.end());
            }
            else
            {
                response->flag = false;
                LOG_INFO("control_points.size is not 6. is path.size()={}",
                         request->path.size());
            }
        }
        else if (request->line_flag == 4) // 圆弧+圆弧
        {
            arc1_.clear();
            arc2_.clear();
            ctrl_pts_.clear();
            merged_.clear();
            if (request->path.size() == 8)
            {
                for (size_t i = 0; i < request->path.size() - 4; i++)
                {
                    start_point_.x = request->path[i].x;
                    start_point_.y = request->path[i].y;
                    start_point_.speed = 0.0;
                    ctrl_pts_.push_back(start_point_);
                }
                arc1_point3d_ = generateBezierUniformPoints(ctrl_pts_, line_arc_point_step_);
                arc1_ = update_trajectory(arc1_point3d_, line_speed_);
                ctrl_pts_.clear();
                arc_speed_ = request->path[4].speed;
                for (size_t i = 4; i < request->path.size(); i++)
                {
                    start_point_.x = request->path[i].x;
                    start_point_.y = request->path[i].y;
                    start_point_.speed = 0.0;
                    ctrl_pts_.push_back(start_point_);
                }
                arc2_point3d_ = generateBezierUniformPoints(ctrl_pts_, line_arc_point_step_);
                arc2_ = update_trajectory(arc1_point3d_, arc_speed_);
                merged_.reserve(arc2_.size() + arc1_.size());              // 预分配内存，提高效率
                merged_.insert(merged_.end(), arc1_.begin(), arc1_.end()); // 先圆弧后直线
                merged_.insert(merged_.end(), arc2_.begin(), arc2_.end());
            }
            else
            {
                response->flag = false;
                LOG_INFO("control_points.size is not 8. is path.size()={}", request->path.size());
            }
        }
        else
        {
            has_line_stop_ = true;
            line1_.clear();
            line2_.clear();
            merged_.clear();
            line1_point3d_.clear();
            start_point_.x = request->path[0].x;
            start_point_.y = request->path[0].y;
            end_point_.x = request->path[1].x;
            end_point_.y = request->path[1].y;
            LOG_INFO("response: start_point_,x={:.4f}, y={:.4f}, z={:.4f}", start_point_.x, start_point_.y, start_point_.speed);
            LOG_INFO("response: end_point_,x={:.4f}, y={:.4f}, speed={:.4f}", end_point_.x, end_point_.y, line_speed_);
            // line1_point3d_ = generateFixedStepPoints(start_point_, end_point_, line_arc_point_step_);
            extend_end_point_ = extendEndPoint(start_point_, end_point_, line_expand_step_); // 延长终点固定距离
            line1_point3d_ = generateFixedStepPoints(start_point_, extend_end_point_, line_arc_point_step_);
            line1_ = update_trajectory(line1_point3d_, line_speed_);
        }
        LOG_INFO("has_line_stop_={},is_rotation_={}", has_line_stop_, is_rotation_);
    }
    else if (request->task.task == byd_custom_msgs::msg::TaskType::ENDPATH)
    {
        line_speed_ = request->path[1].speed;
        v_default_ = line_speed_;
        LOG_INFO("v_default_ : {:.4f},request->task.task: {}", v_default_, request->task.task);
        start_point_.x = request->path[0].x;
        start_point_.y = request->path[0].y;
        end_point_.x = request->path[1].x;
        end_point_.y = request->path[1].y;

        neg_v_flag_ = false;
        is_rotation_ = checkisrotation(start_point_, end_point_, current_xy_);
        // is_start_init_ = checkisstart_init(start_point_, end_point_, current_xy_);
        if (is_rotation_)
        {
            is_rotation_yaw_ = atan2((end_point_.y - start_point_.y), (end_point_.x - start_point_.x)); // calrotationyaw(start_point_,end_point_,current_xy_);
        }
        else
        {
            // 新增：调用feedback_perception_client_发送布尔true
            if (feedback_perception_client_ &&
                feedback_perception_client_->service_is_ready())
            {
                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = true;
                feedback_perception_client_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
                                                                {
                        auto res = future.get();
                        if (res->success)
                        {LOG_INFO("feedback_perception_client_ 发送true成功");}
                        else
                        {LOG_INFO("feedback_perception_client_ 发送true失败");} });
            }
            percerption_start_time_ = std::chrono::steady_clock::now();
        }
        LOG_INFO("is_rotation_ ,current_xy_.speed : {:.4f},{:.4f}", is_rotation_, current_xy_.speed);
    }
    else
    {
        is_rotation_ = false;
        response->flag = false;
    }
    LOG_INFO("handle_task_to_planning_request 任务完成状态: {}", response->flag);
}
// 调用 /task_feedback 服务反馈客户端

int main(int argc, char *argv[])
{
    auto logger = get_logger();
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
