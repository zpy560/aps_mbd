#include "control.hpp"

double PurePursuit::pure_pursuit_control(const geometry_msgs::msg::Point &lookaheadPoint,
                                         const geometry_msgs::msg::Point &currentPosi,
                                         double Ld)
{
  byd_custom_msgs::msg::PathPoints err;
  double lat_err = 0;
  double head_err = 0;
  double R_ = 100;

  // 计算误差
  err.x = lookaheadPoint.x - currentPosi.x;
  err.y = lookaheadPoint.y - currentPosi.y;

  // 横向误差
  lat_err = err.y * std::cos(currentPosi.z) - err.x * std::sin(currentPosi.z);

  // 航向误差
  head_err = std::atan2(err.y, err.x) - currentPosi.z;

  // 归一化航向误差到 (-pi, pi]
  if (head_err > pi)
  {
    head_err -= 2 * pi;
  }
  else if (head_err < -pi)
  {
    head_err += 2 * pi;
  }

  // 计算转弯半径
  if (std::abs(std::sin(head_err)) < 1e-6) // 防止除零
  {
    R_ = 1e6; // 一个很大的半径，接近直线
  }
  else
  {
    R_ = Ld / (2 * std::sin(head_err));
  }

  // 记录日志
  // LOG_INFO("PPC control: Ld={},R={},lat_err={},head_err={},car_pos=({},{},{} rad,{} deg),target=({},{},{})",Ld, R_, lat_err,head_err,currentPosi.x, currentPosi.y, currentPosi.z, currentPosi.z * rad2angle,lookaheadPoint.x, lookaheadPoint.y, lookaheadPoint.z);

  // RCLCPP_INFO(logger_,"PPC control: Ld=%.3f, R=%.4f, lat_err=%.4f, head_err=%.5f, car_pos=(%.3f, %.3f, %.5f rad / %.3f deg), target=(%.3f, %.3f, %.5f)",
  //             Ld, R_, lat_err,head_err,currentPosi.x, currentPosi.y, currentPosi.z, currentPosi.z * rad2angle,lookaheadPoint.x, lookaheadPoint.y, lookaheadPoint.z);

  return R_;
}
