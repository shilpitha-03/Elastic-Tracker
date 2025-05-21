// src/detection/target_ekf/src/target_ekf_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <object_detection_msgs/msg/bounding_boxes.hpp>
#include <rmw/qos_profiles.h>

using std::placeholders::_1;
using std::placeholders::_2;

class TargetEkfNode : public rclcpp::Node
{
public:
  TargetEkfNode()
  : Node("target_ekf")
  {
    // declare + read camera parameters (loaded via YAML in launch)
    declare_parameter<int>("cam_width", 640);
    declare_parameter<int>("cam_height", 480);
    declare_parameter<double>("cam_fx", 346.74048);
    declare_parameter<double>("cam_fy", 349.12708);
    declare_parameter<double>("cam_cx", 322.83899);
    declare_parameter<double>("cam_cy", 234.54992);
    declare_parameter<std::vector<double>>("cam2body_R", std::vector<double>());
    declare_parameter<std::vector<double>>("cam2body_p", std::vector<double>());
    declare_parameter<double>("pitch_thr", 30.0);
    declare_parameter<int>("ekf_rate", 20);

    get_parameter("cam_fx", fx_);
    get_parameter("cam_fy", fy_);
    get_parameter("cam_cx", cx_);
    get_parameter("cam_cy", cy_);
    get_parameter("pitch_thr", pitch_thr_);
    int ekf_rate;
    get_parameter("ekf_rate", ekf_rate);

    // load cam2body transform
    std::vector<double> tmp;
    get_parameter("cam2body_R", tmp);
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(tmp.data());
    get_parameter("cam2body_p", tmp);
    cam2body_p_ = Eigen::Map<const Eigen::Vector3d>(tmp.data());

    // publishers
    target_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("target_odom", 10);
    yolo_odom_pub_   = create_publisher<nav_msgs::msg::Odometry>("yolo_odom", 10);

    // message_filters subscribers & synchronizer
    yolo_sub_.subscribe(this, "yolo", rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, "odom", rmw_qos_profile_default);

    using Policy = message_filters::sync_policies::ApproximateTime<
      object_detection_msgs::msg::BoundingBoxes,
      nav_msgs::msg::Odometry
    >;
    sync_.reset(new message_filters::Synchronizer<Policy>(
      Policy(200), yolo_sub_, odom_sub_));
    sync_->registerCallback(
      std::bind(&TargetEkfNode::update_state_callback, this, _1, _2)
    );

    // EKF predict timer
    ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);
    last_update_stamp_ = now() - rclcpp::Duration::from_seconds(10.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0/ekf_rate),
      std::bind(&TargetEkfNode::predict_state_callback, this)
    );
  }

private:
  // EKF struct unchanged (6‑state version)
  struct Ekf {
    double dt;
    Eigen::MatrixXd A, B, C, Qt, Rt, Sigma, K;
    Eigen::VectorXd x;
    Ekf(double _dt) : dt(_dt) {
      A.setIdentity(6,6);
      Sigma.setZero(6,6);
      B.setZero(6,3);
      C.setZero(3,6);
      A(0,3)=A(1,4)=A(2,5)=dt;
      double t2=dt*dt/2;
      for(int i=0;i<3;i++){ B(i,i)=t2; B(i+3,i)=dt; C(i,i)=1; }
      K=C;
      Qt.setIdentity(3,3);
      Rt.setIdentity(3,3);
      Qt(0,0)=Qt(1,1)=4; Qt(2,2)=1;
      Rt(0,0)=Rt(1,1)=Rt(2,2)=0.1;
      x.setZero(6);
    }
    void predict() {
      x = A*x;
      Sigma = A*Sigma*A.transpose() + B*Qt*B.transpose();
    }
    void reset(const Eigen::Vector3d &z) {
      x.head(3)=z; x.tail(3).setZero(); Sigma.setZero();
    }
    bool checkValid(const Eigen::Vector3d &z) const {
      auto Ktmp = Sigma*C.transpose()*(C*Sigma*C.transpose()+Rt).inverse();
      auto xtmp = x + Ktmp*(z - C*x);
      return xtmp.tail(3).norm() <= 4.0;
    }
    void update(const Eigen::Vector3d &z) {
      K = Sigma*C.transpose()*(C*Sigma*C.transpose()+Rt).inverse();
      x = x + K*(z - C*x);
      Sigma = Sigma - K*C*Sigma;
    }
    Eigen::Vector3d pos() const { return x.head(3); }
    Eigen::Vector3d vel() const { return x.tail(3); }
  };

  // callbacks
  void predict_state_callback() {
    double dt = (now() - last_update_stamp_).seconds();
    if (dt < 2.0) {
      ekfPtr_->predict();
    } else {
      RCLCPP_WARN(get_logger(), "too long without update");
      return;
    }
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.pose.pose.position.x = ekfPtr_->pos().x();
    msg.pose.pose.position.y = ekfPtr_->pos().y();
    msg.pose.pose.position.z = ekfPtr_->pos().z();
    msg.twist.twist.linear.x = ekfPtr_->vel().x();
    msg.twist.twist.linear.y = ekfPtr_->vel().y();
    msg.twist.twist.linear.z = ekfPtr_->vel().z();
    msg.pose.pose.orientation.w = 1.0;
    target_odom_pub_->publish(msg);
  }

  void update_state_callback(
    const object_detection_msgs::msg::BoundingBoxes::ConstSharedPtr bboxes_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
  {
    // compute camera‐frame target position
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

    auto &y = bboxes_msg->bounding_boxes.front();
    double height = (y.ymax - y.ymin);
    if (y.ymin < 30.0 || y.ymax > cam_height_-30) {
      RCLCPP_ERROR(get_logger(), "bbox out of range");
      return;
    }
    double depth = 0.7 / height * fy_;
    double X = ((y.xmin+y.xmax)/2.0 - cx_) * depth / fx_;
    double Y = ((y.ymin+y.ymax)/2.0 - cy_) * depth / fy_;
    Eigen::Vector3d p_cam{X, Y, depth};
    Eigen::Vector3d p_world = cam_q * p_cam + cam_p;

    // publish yolo odom
    auto ymsg = nav_msgs::msg::Odometry();
    ymsg.header.stamp = bboxes_msg->header.stamp;
    ymsg.header.frame_id = "world";
    ymsg.pose.pose.position.x = p_world.x();
    ymsg.pose.pose.position.y = p_world.y();
    ymsg.pose.pose.position.z = p_world.z();
    ymsg.pose.pose.orientation.w = 1.0;
    yolo_odom_pub_->publish(ymsg);

    // EKF update
    double dt = (now() - last_update_stamp_).seconds();
    if (dt > 3.0) {
      ekfPtr_->reset(p_world);
      RCLCPP_WARN(get_logger(), "ekf reset");
    } else if (ekfPtr_->checkValid(p_world)) {
      ekfPtr_->update(p_world);
    } else {
      RCLCPP_ERROR(get_logger(), "update invalid");
      return;
    }
    last_update_stamp_ = now();
  }

  // members
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_, yolo_odom_pub_;
  message_filters::Subscriber<object_detection_msgs::msg::BoundingBoxes> yolo_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      object_detection_msgs::msg::BoundingBoxes,
      nav_msgs::msg::Odometry
    >
  >> sync_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Ekf> ekfPtr_;
  rclcpp::Time last_update_stamp_;

  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;
  double fx_, fy_, cx_, cy_, pitch_thr_;
  int cam_width_, cam_height_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetEkfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
