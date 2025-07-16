#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "byd_custom_msgs/srv/bizer_to_mcu.hpp"  // 服务头文件
#include "geometry_msgs/msg/point.hpp"

class BizerToMcuServer : public rclcpp::Node
{
public:
  BizerToMcuServer()
  : Node("bizer_to_mcu_server")
  {
    // 创建服务，绑定成员函数作为回调
    service_ = this->create_service<byd_custom_msgs::srv::BizerToMcu>(
      "bizer_to_mcu",std::bind(&BizerToMcuServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Ready to process control points.");
  }

private:
  void handle_service(
    const std::shared_ptr<byd_custom_msgs::srv::BizerToMcu::Request> request,
    std::shared_ptr<byd_custom_msgs::srv::BizerToMcu::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received control points from client:");
    for (size_t i = 0; i < request->contr_points.size(); ++i) {
      const auto & pt = request->contr_points[i];
      RCLCPP_INFO(this->get_logger(), "Point %zu: x=%.3f, y=%.3f, z=%.3f",
                  i, pt.x, pt.y, pt.z);
    }
    // 模拟业务处理
    response->flag = true;
    RCLCPP_INFO(this->get_logger(), "Processed control points successfully.");
  }

  rclcpp::Service<byd_custom_msgs::srv::BizerToMcu>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BizerToMcuServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
