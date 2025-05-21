// src/visualize_history_path.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Geometry>

using std::placeholders::_1;

class VisualizeHistoryPath : public rclcpp::Node
{
public:
  VisualizeHistoryPath()
  : Node("visualize_history_path")
  {
    remove_floor_ceil_ = this->declare_parameter<bool>("remove_floor_ceil", false);

    drone_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("history_drone_pose", 100);
    target_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("history_target_pose", 100);
    lines_pub_      = this->create_publisher<visualization_msgs::msg::Marker>("drone_target_link_line", 100);
    spring_pub_     = this->create_publisher<nav_msgs::msg::Path>("spring", 100);

    cmd_sub_    = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
                     "/position_cmd", 100, std::bind(&VisualizeHistoryPath::cmd_callback, this, _1));
    target_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                     "/object_odom_dtc2brig", 100, std::bind(&VisualizeHistoryPath::target_odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "visualize_history_path node initialized");
  }

private:
  void cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position = cmd->position;
    drone_path_.header.frame_id = "world";
    drone_path_.poses.push_back(pose);
    drone_path_pub_->publish(drone_path_);
  }

  void target_odom_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // assume msg contains one pose? if actually Odometry, change type accordingly
    auto pose = msg->poses.back();
    target_path_.header.frame_id = "world";
    target_path_.poses.push_back(pose);
    target_path_pub_->publish(target_path_);

    // spring visualization
    if (!drone_path_.poses.empty()) {
      Eigen::Vector3d p0{
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z};
      Eigen::Vector3d p1{
        drone_path_.poses.back().pose.position.x,
        drone_path_.poses.back().pose.position.y,
        drone_path_.poses.back().pose.position.z};
      Eigen::Vector3d dp = p1 - p0;
      Eigen::Vector3d dx = dp.normalized();
      Eigen::Vector3d dy = Eigen::Vector3d(0,0,1).cross(dx);
      Eigen::Vector3d dz = dx.cross(dy);

      nav_msgs::msg::Path spring;
      spring.header.frame_id = "world";
      for (double t=0; t<10*2*M_PI; t+=0.1) {
        double y = 0.2*std::cos(t), z = 0.2*std::sin(t);
        Eigen::Vector3d p = p0 + dp*(t/(10*2*M_PI)) + z*dz + y*dy;
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        spring.poses.push_back(ps);
      }
      spring_pub_->publish(spring);
    }

    // line marker
    visualization_msgs::msg::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = now();
    line_list.ns = "lines";
    line_list.id = 0;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.scale.x = 0.03;
    line_list.color.r = 0.0; line_list.color.g = 1.0; line_list.color.b = 0.1; line_list.color.a = 0.5;

    geometry_msgs::msg::Point p_start = pose.pose.position;
    geometry_msgs::msg::Point p_end = drone_path_.poses.back().pose.position;
    line_list.points.push_back(p_start);
    line_list.points.push_back(p_end);
    lines_pub_->publish(line_list);
  }

  bool remove_floor_ceil_;
  nav_msgs::msg::Path drone_path_, target_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr drone_path_pub_, target_path_pub_, spring_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lines_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualizeHistoryPath>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
