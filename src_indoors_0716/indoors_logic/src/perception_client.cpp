
#include "rclcpp/rclcpp.hpp"
#include "byd_custom_msgs/srv/perception_to_bizer.hpp"
#include <cstdlib>


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 10) {
        RCLCPP_INFO(rclcpp::get_logger("add_two_ints_client"), "Usage: add_two_ints_client X Y");
        return 1;
    }

    auto node = rclcpp::Node::make_shared("add_two_ints_client");
    auto client = node->create_client<byd_custom_msgs::srv::PerceptionToBizer>("/shelf_pose");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("add_two_ints_client"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
    }
    auto request = std::make_shared<byd_custom_msgs::srv::PerceptionToBizer::Request>();
    // byd_custom_msgs::srv::PerceptionToBizer::Request request;
    geometry_msgs::msg::Pose middle_pose;
    middle_pose.position.x = atoll(argv[1]);
    middle_pose.position.y = atoll(argv[2]);
    middle_pose.position.z = atoll(argv[3]);
    middle_pose.orientation.x = 0.0;
    middle_pose.orientation.y = 0.0;
    middle_pose.orientation.z = 0.0;
    middle_pose.orientation.w = 1.0;

    request->send_points[0].pose = middle_pose;
    middle_pose.position.x = atoll(argv[4]);
    middle_pose.position.y = atoll(argv[5]);
    middle_pose.position.z = atoll(argv[6]);
    middle_pose.orientation.x = 0.0;
    middle_pose.orientation.y = 0.0;
    middle_pose.orientation.z = 0.0;
    middle_pose.orientation.w = 1.0;
    request->send_points[1].pose = middle_pose;

    middle_pose.position.x = atoll(argv[7]);
    middle_pose.position.y = atoll(argv[8]);
    middle_pose.position.z = atoll(argv[9]);
    middle_pose.orientation.x = 0.0;
    middle_pose.orientation.y = 0.0;
    middle_pose.orientation.z = 0.0;
    middle_pose.orientation.w = 1.0;
    request->send_points[2].pose = middle_pose;

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("add_two_ints_client"), "Sum: %d",result.get()->flag);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("add_two_ints_client"), "Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}
