// src/detection/target_ekf/src/target_ekf_sim_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <target_ekf/target_ekf.hpp>  // your EKF + euler2quaternion, quaternion2euler

using std::placeholders::_1;
using std::placeholders::_2;

class TargetEkfSimNode : public rclcpp::Node
{
public:
  TargetEkfSimNode()
  : Node("target_ekf_sim")
  {
    // declare parameters
    declare_parameter<double>("cam_fx", 1397.22);
    declare_parameter<double>("cam_fy", 1397.22);
    declare_parameter<double>("cam_cx", 960.00);
    declare_parameter<double>("cam_cy", 540.00);
    declare_parameter<int>("cam_width", 1920);
    declare_parameter<int>("cam_height", 1080);
    declare_parameter<std::vector<double>>("cam2body_R", std::vector<double>({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}));
    declare_parameter<std::vector<double>>("cam2body_p", std::vector<double>({ 0.13233, 0.0, 0.26078 }));
    declare_parameter<double>("pitch_thr", 30.0);
    declare_parameter<bool>("check_fov", false);
    declare_parameter<int>("ekf_rate", 20);

    // get parameters
    get_parameter("cam_fx", fx_);
    get_parameter("cam_fy", fy_);
    get_parameter("cam_cx", cx_);
    get_parameter("cam_cy", cy_);
    get_parameter("cam_width", width_);
    get_parameter("cam_height", height_);
    get_parameter("pitch_thr", pitch_thr_);
    get_parameter("check_fov", check_fov_);
    int ekf_rate; get_parameter("ekf_rate", ekf_rate);

    // load cam2body transform
    std::vector<double> tmp;
    get_parameter("cam2body_R", tmp);
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(tmp.data());
    get_parameter("cam2body_p", tmp);
    cam2body_p_ = Eigen::Map<const Eigen::Vector3d>(tmp.data());

    // publishers
    // target_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("target_odom", 10);
    // yolo_odom_pub_   = create_publisher<nav_msgs::msg::Odometry>("yolo_odom", 10);
    // publishers (BEST_EFFORT, depth=1)
    rclcpp::QoS be_qos(1);
    be_qos.best_effort();
    target_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("target_odom", be_qos);
    yolo_odom_pub_   = create_publisher<nav_msgs::msg::Odometry>("yolo_odom",   be_qos);


    // // message_filters subscribers & synchronizer (two Odometry topics)
    // yolo_sub_.subscribe(this, "target_pose", rmw_qos_profile_default);
    // odom_sub_.subscribe(this, "odometry", rmw_qos_profile_default);
    // message_filters subscribers & synchronizer (two Odometry topics, BEST_EFFORT)
    yolo_sub_.subscribe(this, "target", rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, "odometry",    rmw_qos_profile_sensor_data);
    using Policy = message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>;
    sync_.reset(new message_filters::Synchronizer<Policy>(
      Policy(200), yolo_sub_, odom_sub_));
    sync_->registerCallback(
      std::bind(&TargetEkfSimNode::update_state_callback, this, _1, _2)
    );

    // EKF and timer
    ekfPtr_ = std::make_shared<Ekf>(1.0/ekf_rate);
    last_update_stamp_ = now() - rclcpp::Duration::from_seconds(10.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0/ekf_rate),
      std::bind(&TargetEkfSimNode::predict_state_callback, this)
    );
  }

private:
  // EKF struct and quaternion helpers are in target_ekf.hpp — no change needed

  void predict_state_callback()
  {
    last_update_stamp_ = now();
    double dt = (now() - last_update_stamp_).seconds();
    if (dt < 2.0) {
      ekfPtr_->predict();
    } else {
      RCLCPP_WARN(get_logger(), "[ekf] too long time no update");
      return;
    }
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.pose.pose.position.x = ekfPtr_->pos().x();
    msg.pose.pose.position.y = ekfPtr_->pos().y();
    msg.pose.pose.position.z = ekfPtr_->pos().z();
    msg.twist.twist.linear.x = ekfPtr_->vel().x();
    msg.twist.twist.linear.y = ekfPtr_->vel().y();
    msg.twist.twist.linear.z = ekfPtr_->vel().z();
    Eigen::Vector3d rpy = ekfPtr_->rpy();
    Eigen::Quaterniond q = euler2quaternion(rpy);
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    target_odom_pub_->publish(msg);
  }

  void update_state_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr target_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
  {
    // world→camera  
    Eigen::Vector3d odom_p{
      odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z
    };
    Eigen::Quaterniond odom_q{
      odom_msg->pose.pose.orientation.w,
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z
    };
    Eigen::Vector3d cam_p = odom_q * cam2body_p_ + odom_p;
    Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

    // target pose from first topic
    Eigen::Vector3d p{
      target_msg->pose.pose.position.x,
      target_msg->pose.pose.position.y,
      target_msg->pose.pose.position.z
    };
    Eigen::Quaterniond q{
      target_msg->pose.pose.orientation.w,
      target_msg->pose.pose.orientation.x,
      target_msg->pose.pose.orientation.y,
      target_msg->pose.pose.orientation.z
    };
    Eigen::Vector3d rpy = quaternion2euler(q);

    // optional FOV check
    if (check_fov_) {
      Eigen::Vector3d in_body = cam_q.inverse() * (p - cam_p);
      if (in_body.z() < 0.1 || in_body.z() > 5.0) return;
      double u = in_body.x()*fx_/in_body.z() + cx_;
      double v = in_body.y()*fy_/in_body.z() + cy_;
      if (u<0 || u>width_ || v<0 || v>height_) return;
    }

    // EKF update
    double dt = (now() - last_update_stamp_).seconds();
    if (dt > 5.0) {
      ekfPtr_->reset(p, rpy);
      RCLCPP_WARN(get_logger(), "[ekf] reset");
    } else if (ekfPtr_->update(p, rpy)) {
      // updated
    } else {
      RCLCPP_ERROR(get_logger(), "[ekf] update invalid");
      return;
    }
    last_update_stamp_ = now();
  }

  // members
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_, yolo_odom_pub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> yolo_sub_, odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry, nav_msgs::msg::Odometry
    >
  >> sync_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Ekf> ekfPtr_;
  rclcpp::Time last_update_stamp_;

  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;
  double fx_, fy_, cx_, cy_, pitch_thr_;
  int width_, height_;
  bool check_fov_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetEkfSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
