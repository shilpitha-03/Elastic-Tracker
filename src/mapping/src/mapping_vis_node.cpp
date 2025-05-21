#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <quadrotor_msgs/msg/occ_map3d.hpp>
#include <mapping/mapping.h>

using std::placeholders::_1;

class MappingVisNode : public rclcpp::Node {
public:
  MappingVisNode()
  : Node("mapping_vis") {
    // Declare and get parameter
    this->declare_parameter<bool>("remove_floor_ceil", false);
    remove_floor_ceil_ = this->get_parameter("remove_floor_ceil").as_bool();

    // Publishers
    gridmap_vs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vs_gridmap", 1);
    gridmap_inflate_vs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vs_gridmap_inflate", 1);

    // Subscribers
    gridmap_sub_ = this->create_subscription<quadrotor_msgs::msg::OccMap3d>(
      "gridmap", 1,
      std::bind(&MappingVisNode::gridmap_callback, this, _1));

    gridmap_inflate_sub_ = this->create_subscription<quadrotor_msgs::msg::OccMap3d>(
      "gridmap_inflate", 1,
      std::bind(&MappingVisNode::gridmap_inflate_callback, this, _1));
  }

private:
  void gridmap_callback(const quadrotor_msgs::msg::OccMap3d::SharedPtr msgPtr) {
    mapping::OccGridMap gridmap;
    gridmap.from_msg(*msgPtr);
    sensor_msgs::msg::PointCloud2 pc;
    gridmap.occ2pc(pc);
    pc.header.stamp = this->get_clock()->now();
    pc.header.frame_id = "world";
    gridmap_vs_pub_->publish(pc);
  }

  void gridmap_inflate_callback(const quadrotor_msgs::msg::OccMap3d::SharedPtr msgPtr) {
    mapping::OccGridMap gridmap;
    gridmap.from_msg(*msgPtr);
    sensor_msgs::msg::PointCloud2 pc;
    if (remove_floor_ceil_) {
      gridmap.occ2pc(pc, 0.5, 2.5);
    } else {
      gridmap.occ2pc(pc);
    }
    pc.header.stamp = this->get_clock()->now();
    pc.header.frame_id = "world";
    gridmap_inflate_vs_pub_->publish(pc);
  }

  bool remove_floor_ceil_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gridmap_vs_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gridmap_inflate_vs_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::OccMap3d>::SharedPtr gridmap_inflate_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingVisNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
