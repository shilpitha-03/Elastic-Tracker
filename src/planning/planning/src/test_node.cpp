#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <prediction/prediction.hpp>
#include <visualization/visualization.hpp>
#include <mapping/mapping.hpp>

using std::placeholders::_1;

class TestNode : public rclcpp::Node {
public:
  TestNode()
  : Node("test_node")
  {
    // subscriptions
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    gridmap_sub_ = this->create_subscription<quadrotor_msgs::msg::OccMap3d>(
      "gridmap_inflate", qos,
      std::bind(&TestNode::gridmap_callback, this, _1));
    trigger_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "triger", qos,
      std::bind(&TestNode::triger_callback, this, _1));

    // timer (0.3s)
    test_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(300),
      std::bind(&TestNode::testCallback, this));

    // core objects (pass node handle)
    prePtr_ = std::make_shared<prediction::Predict>(*this);
    visPtr_ = std::make_shared<visualization::Visualization>(*this);

    RCLCPP_WARN(this->get_logger(), "[TEST NODE]: ready.");
  }

private:
  void gridmap_callback(const quadrotor_msgs::msg::OccMap3d::SharedPtr msg) {
    if (map_received_) return;
    map_.from_msg(*msg);
    prePtr_->setMap(map_);
    RCLCPP_WARN(this->get_logger(), "[TEST NODE] GLOBAL MAP RECEIVED");
    map_received_ = true;
  }

  void triger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_p_ << msg->pose.position.x, msg->pose.position.y, 1.0;
    // fixed velocity
    target_v_ << 1.0, 0.0, 0.0;

    std::vector<Eigen::Vector3d> predict_path;
    RCLCPP_INFO(this->get_logger(),
                "target_p: [%f, %f, %f]",
                target_p_.x(), target_p_.y(), target_p_.z());
    prePtr_->predict(target_p_, target_v_, predict_path);
    visPtr_->visualize_pointcloud(predict_path, "future_pts");
  }

  void testCallback() {
    // you can add periodic tests here
  }

  // subs/timer
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr test_timer_;

  // logic
  mapping::OccGridMap map_;
  bool map_received_ = false;
  Eigen::Vector3d target_p_, target_v_;
  std::shared_ptr<prediction::Predict> prePtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}
