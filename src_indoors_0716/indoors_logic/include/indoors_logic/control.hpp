#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "byd_custom_msgs/msg/path_points.hpp"
#include "geometry_msgs/msg/point.hpp"

constexpr double pi = 3.14159265358979323846;
constexpr double rad2angle = 180.0 / pi;

class PurePursuit
{
public:
  PurePursuit(rclcpp::Logger logger = rclcpp::get_logger("PurePursuit"))
      : logger_(logger)
  {
  }

  /**
   * @brief 纯追踪控制计算转弯半径
   * @param lookaheadPoint 目标点位置
   * @param currentPosi  当前车辆位置
   * @param Ld  预瞄距离
   * @return 转弯半径 R
   */
  double pure_pursuit_control(
      const geometry_msgs::msg::Point &lookaheadPoint,
      const geometry_msgs::msg::Point &currentPosi,
      double Ld);

private:
  rclcpp::Logger logger_;
};

#endif // CONTROL_HPP_
