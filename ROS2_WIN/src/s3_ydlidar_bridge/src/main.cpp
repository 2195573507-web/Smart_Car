#include "rclcpp/rclcpp.hpp"
#include "s3_ydlidar_bridge/bridge_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s3_ydlidar_bridge::BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
