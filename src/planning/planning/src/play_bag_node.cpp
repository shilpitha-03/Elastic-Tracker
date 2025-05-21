#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <quadrotor_msgs/msg/replan_state.hpp>
#include <wr_msg/wr_msg.hpp>

using std::placeholders::_1;

class PlayBagNode : public rclcpp::Node {
public:
  PlayBagNode()
  : Node("play_bag_node")
  {
    // QoS: reliable, keep last
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    sub_ = this->create_subscription<quadrotor_msgs::msg::ReplanState>(
      "replanState", qos,
      std::bind(&PlayBagNode::replan_state_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "play_bag_node started");
  }

private:
  void replan_state_callback(const quadrotor_msgs::msg::ReplanState::SharedPtr msg) {
    if (msg->state == 2) {
      // write to debug file
      auto pkg_path = ament_index_cpp::get_package_share_directory("planning");
      wr_msg::writeMsg(*msg, pkg_path + "/../../../debug/replan_state.bin");
      RCLCPP_FATAL(this->get_logger(), "ReplanState==2, aborting");
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<quadrotor_msgs::msg::ReplanState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlayBagNode>());
  rclcpp::shutdown();
  return 0;
}
