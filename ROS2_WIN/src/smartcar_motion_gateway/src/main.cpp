#include "smartcar_motion_gateway/gateway_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smartcar_motion_gateway::GatewayNode>());
  rclcpp::shutdown();
  return 0;
}
