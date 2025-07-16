#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>
#include <vector>

#include "bezier_model.hpp"
#include "bezier_three.hpp"
#include "byd_custom_msgs/msg/control_res.hpp"
#include "byd_custom_msgs/msg/motion_state.hpp"
#include "byd_custom_msgs/msg/trajectory.hpp"
#include "byd_custom_msgs/srv/bizer_to_mcu.hpp"
#include "byd_custom_msgs/srv/control_to_task.hpp"
#include "byd_custom_msgs/srv/controller.hpp"
#include "byd_custom_msgs/srv/perception_to_bizer.hpp"
#include "byd_custom_msgs/srv/status.hpp"
#include "byd_custom_msgs/srv/taskcs.hpp"
#include "control.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pose.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <functional>
#include <iomanip>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "Logger/Logger.h"
#include "status.hpp"

using PerceptionToBizer = byd_custom_msgs::srv::PerceptionToBizer;
using BizerToMcu = byd_custom_msgs::srv::BizerToMcu;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    void initial();

private:
    // void trajectory_callback(const byd_custom_msgs::msg::Trajectory::SharedPtr
    // msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timerCallback();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void obstacle_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void motion_state_callback(const byd_custom_msgs::msg::MotionState::SharedPtr msg);
    void handle_remote_request(const std::shared_ptr<byd_custom_msgs::srv::Status::Request> request,
                               std::shared_ptr<byd_custom_msgs::srv::Status::Response> response);
    void handle_service_remote_control_request(const std::shared_ptr<byd_custom_msgs::srv::Controller::Request> request,
        std::shared_ptr<byd_custom_msgs::srv::Controller::Response> response);
    void handle_task_to_planning_request(const std::shared_ptr<byd_custom_msgs::srv::Taskcs::Request> request,
        std::shared_ptr<byd_custom_msgs::srv::Taskcs::Response> response);
    double normalizeAngle(double angle);
    void save_log_to_file(const std::string &log_msg, bool is_save_to_file);
    void call_feedback_service(bool is_remote);
    void PerceptionToBizer_callback(const std::shared_ptr<PerceptionToBizer::Request> req,std::shared_ptr<PerceptionToBizer::Response> res);
    int ternarySearchClosestIndex(const byd_custom_msgs::msg::PathPoints &p,const byd_custom_msgs::msg::Trajectory msg);
    double distanceSquared_node(const byd_custom_msgs::msg::PathPoints &a,const geometry_msgs::msg::Point &b);
    bool isPoseStampedValid(const geometry_msgs::msg::PoseStamped &pose_stamped);
    bool checkisrotation(const Point3D &start_point, const Point3D &end_point,const byd_custom_msgs::msg::PathPoints &current_xy);
    uint8_t checkisstart_init(const Point3D &start_point, const Point3D &end_point,const byd_custom_msgs::msg::PathPoints &current_xy);
    bool isProjectionOnSegment(const Eigen::Vector2d &start,const Eigen::Vector2d &end,
                               const Eigen::Vector2d &point);
    double controlAngularVelocity(double error, double omega, double wmax,double wacc, double dt);
    double clamp(double val, double min_val, double max_val);
    std::pair<double, double> vwprocess(double car_v, double car_w,double desire_v, double desire_w, double max_acc_v, double max_acc_w,double dt);
    double liftvprocess(double lift_height_current, double lift_height_desire,double max_v, double max_a, double dt, double epsilon,
                        double startstopdis);
    double roatvprocess(double lift_height_current, double lift_height_desire,double max_v, double max_a, double dt, double epsilon,
                        double startstopdis);
    Point3D vectorDiff(const Point3D &a, const Point3D &b);
    double vectorLength(const Point3D &v);
    Point3D normalizeVector(const Point3D &v);
    Point3D extendEndPoint(const Point3D &start, const Point3D &end,const double extend_distance);
    // 订阅者
    rclcpp::Subscription<byd_custom_msgs::msg::Trajectory>::SharedPtr
        trajectory_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<byd_custom_msgs::msg::MotionState>::SharedPtr motion_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr obstacle_status_sub_;
    // 发布者
    rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr car_op_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<byd_custom_msgs::srv::Status>::SharedPtr remote_service_;
    rclcpp::Service<byd_custom_msgs::srv::Controller>::SharedPtr remote_control_service_;
    rclcpp::Service<byd_custom_msgs::srv::Taskcs>::SharedPtr task_to_planning_service_;
    rclcpp::Client<byd_custom_msgs::srv::ControlToTask>::SharedPtr feedback_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr feedback_perception_client_;

    rclcpp::Service<PerceptionToBizer>::SharedPtr service_;
    //   rclcpp::Client<BizerToMcu>::SharedPtr client_;

    // 纯追踪控制对象
    PurePursuit ppc_;

    std::string log_msg_; // 日志消息

    // 预瞄距离
    double Ld_;
    double Ld_start_;
    int closest_index_;
    long unsigned int target_index_;
    double v_default_;         // 单位默认 m/s
    double endpath_v_default_; // 末端终点默认速度，单位默认 m/s
    double current_yaw_;
    double line_speed_; // 直线速度
    double arc_speed_;  // 弧线速度

    // 缓存定位数据（线程安全）
    std::mutex pose_mutex_;
    std_msgs::msg::Int32 obstacle_status_;
    geometry_msgs::msg::Point current_pose_;
    byd_custom_msgs::msg::PathPoints current_xy_;
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PoseStamped current_pose_to_odom_;
    // byd_custom_msgs::msg::PathPoints current_xy_;
    byd_custom_msgs::msg::MotionState current_motion_state_;
    bool pose_received_;
    bool odom_received_;
    bool is_rotation_;   // 是否有原地旋转
    uint8_t is_start_init_; // 是否开始初始化--跟踪到起点
    double is_rotation_yaw_;
    bool trajectory_received_; // 接收task客户端的轨迹触发
    bool remote_control_valid_;
    bool has_line_stop_; // 任务是否有直线停车点
    bool neg_v_flag_;
    bool is_save_to_file_;                                         // 是否保存日志到文件
    bool use_odom_;                                                // 是否使用odom数据
    int out_end_traj_;
    bool end_out_flag_;
    double distance_startend_;
    bool rec_task_flag_;                                           // 是否接收task
    std::chrono::steady_clock::time_point percerption_start_time_; // 感知开始时间
    std::chrono::steady_clock::time_point percerption_end_time_;   // 感知结束时间
    std::chrono::steady_clock::time_point calendpath_start_time_;  // 计算末端路径时间
    std::chrono::steady_clock::time_point calendpath_end_time_;    // 计算末端路径结束时间
    double second_dec_;
    double first_dec_;

    bool endpath_run_;
    // 填充控制消息并发布
    byd_custom_msgs::msg::ControlRes control_msg_;
    std::vector<geometry_msgs::msg::Point> end_pose_; // 感知计算后的优化样条关键点
    double line_arc_point_step_;                      // 直线弧线离散点数
    int line_expand_step_;                            // 直线延长固定长度 单位mm
    int rotation_stop_;                               //
    int rotation_stop_num_;                           //
    int num_task_;                                    // 任务数
    double cycle_time_;                               // 控制周期时间
    double cargoalspeed_;                             // 底盘速度
    double cargoalwspeed_;                            // 底盘角速度
    std::pair<double, double> current_v_w_;           // 当前速度和角速度
    double v_car;                                     // 底盘当前速度
    double w_car;                                     // 底盘当前角速度
    double calcu_w;                                   // 计算当前底盘角速度
    double carmaxaccel_;
    double carmaxaccel_second_;
    double carmaxaccel_first_;
    double carmax_autoaccel_;
    double carmaxw_; // 底盘最大加速度
    double startstopdis_;
    // double carmaxdecel_;                                 // 底盘最大减速度
    double carmaxwaccel_; // 底盘最大角加速度
    double carmaxwaccel_second_; // 底盘最大角加速度
    double carmaxwaccel_first_; // 底盘最大角加速度

    double carmaxw_startaccel_;
    // double carmaxwdecel_;                                // 底盘最大角减速度
    double startstop_distance_;
    double distance_goal_; // 起停距离
    bool isreach_endpoint_flag_;
    double error;
    double omega;
    bool start_roat_init_;  
    bool is_start_rotation_; // 是否开始旋转
    bool is_stop_rotation_;  // 是否停止旋转
    bool start_stop_flag_;
    double stop_distance_; //                             // 起停标志位; //
                           //                             是否停止旋转
    bool start_line_init_; // 是否开始直线初始化
    double start_v_ ;
    double lift_goal_;     // 目标高度
    double lift_speed_;    // 升降速度
    double lift_accel_;    // 升降加速度
    double epsilon_;       // 死区误差即停止
    double epsilon_roat;   // 死区误差即停止
    // epsilon_roat;
    double lift_max_speed_;                              // 升降最大速度
    double lift_max_accel_;                              // 升降最大加速度
    double lift_max_decel_;                              // 升降最大减速度
    double rotate_goal_;                                 // 托盘旋转目标角度
    double rotate_speed_;                                // 托盘旋转速度
    double rotate_accel_;                                // 托盘旋转加速度
    double rotate_max_speed_;                            // 托盘旋转最大速度
    double rotate_max_accel_;                            // 托盘旋转最大加速度
    double rotate_max_decel_;                            // 托盘旋转最大减速度
    std::vector<geometry_msgs::msg::Point> line1_;       // 任务发布过来的直线离散点1
    std::vector<geometry_msgs::msg::Point> startline1_;       // 任务发布过来的直线离散点1
    std::vector<Point3D> line1_point3d_;                 // 任务发布过来的直线离散点1
    std::vector<Point3D> startline_point3d_;                 // 任务发布过来的直线离散点1
    std::vector<Point3D> line2_point3d_;                 // ; // 任务发布过来的直线离散点1
    std::vector<geometry_msgs::msg::Point> line2_;       // 任务发布过来的直线离散点2
    std::vector<Point3D> arc1_point3d_;                  // ; // 任务发布过来的直线离散点1
    std::vector<Point3D> arc2_point3d_;                  // 任务发布过来的直线离散点1
    std::vector<geometry_msgs::msg::Point> arc1_;        // 任务发布过来的弧线离散点1
    std::vector<geometry_msgs::msg::Point> arc2_;        // 任务发布过来的弧线离散点2
    std::vector<geometry_msgs::msg::Point> merged_;      // 弧线与直线融合数据点
    std::vector<Point3D> ctrl_pts_;                      // 弧线关键控制点数据
    Point3D start_point_;                                // 感知起点坐标
    Point3D is_start_end_;                                // 初始化停止点
    double is_start_yaw_;
    Point3D end_point_;                                  // 感知终点坐标
    Point3D extend_end_point_;                           // 直线终点延长坐标
    byd_custom_msgs::msg::Trajectory end_trajectory_;    // 关键点离散成轨迹点indoors_msgs::msg::Trajectory
    byd_custom_msgs::msg::Trajectory normal_trajectory_; // 关键点离散成轨迹点indoors_msgs::msg::Trajectory
    byd_custom_msgs::msg::TaskType Task_msg_;
    byd_custom_msgs::srv::Controller remote_controller_;
    char remote_under_direction_;
    double remote_linear_v_;
    double remote_angular_v_;
    char remote_upper_direction_;
    double remote_lift_v_;
    double remote_rotate_v_;
    //   byd_custom_msgs::srv::ControlToTask control_to_task_msg_;
    //   CarOpStatus car_op_status_; // add car operation status
    std_msgs::msg::Int32 car_op_status_;
};

#endif // CONTROL_NODE_HPP_
