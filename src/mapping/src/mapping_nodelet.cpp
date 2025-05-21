// mapping/src/mapping_nodelet.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mapping/mapping.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/msg/occ_map3d.hpp>

#include <thread>
#include <atomic>

namespace mapping {

using sensor_msgs::msg::Image;
using nav_msgs::msg::Odometry;
using quadrotor_msgs::msg::OccMap3d;
using std::placeholders::_1;
using std::placeholders::_2;

typedef message_filters::sync_policies::ApproximateTime<Image, Odometry> ImageOdomSyncPolicy;
typedef message_filters::Synchronizer<ImageOdomSyncPolicy> ImageOdomSynchronizer;

struct CamConfig {
  double rate, range, depth_scaling_factor;
  int width, height;
  double fx, fy, cx, cy;
};

class MappingNodelet : public rclcpp::Node {
public:
  MappingNodelet(const rclcpp::NodeOptions &options)
  : Node("mapping_nodelet", options)
  {
    // parameters
    declare_parameter<double>("camera_rate", 30.0);
    declare_parameter<double>("camera_range", 10.0);
    declare_parameter<int>("cam_width", 640);
    declare_parameter<int>("cam_height", 480);
    declare_parameter<double>("cam_fx", 346.74);
    declare_parameter<double>("cam_fy", 349.13);
    declare_parameter<double>("cam_cx", 322.84);
    declare_parameter<double>("cam_cy", 234.55);
    declare_parameter<double>("depth_scaling_factor", 1000.0);

    declare_parameter<int>("down_sample_factor", 2);
    declare_parameter<double>("depth_filter_tolerance", 0.15);
    declare_parameter<double>("depth_filter_mindist", 0.2);
    declare_parameter<int>("depth_filter_margin", 2);

    declare_parameter<int>("p_min", -199);
    declare_parameter<int>("p_max", 220);
    declare_parameter<int>("p_hit", 62);
    declare_parameter<int>("p_mis", 62);
    declare_parameter<int>("p_occ", 139);
    declare_parameter<int>("p_def", -199);

    declare_parameter<double>("resolution", 0.15);
    declare_parameter<int>("local_x", 20);
    declare_parameter<int>("local_y", 20);
    declare_parameter<int>("local_z", 5);
    declare_parameter<int>("inflate_size", 1);

    declare_parameter<bool>("use_global_map", false);
    declare_parameter<bool>("use_mask", false);

    // read into members
    get_parameter("camera_rate", camConfig_.rate);
    get_parameter("camera_range", camConfig_.range);
    get_parameter("cam_width", camConfig_.width);
    get_parameter("cam_height", camConfig_.height);
    get_parameter("cam_fx", camConfig_.fx);
    get_parameter("cam_fy", camConfig_.fy);
    get_parameter("cam_cx", camConfig_.cx);
    get_parameter("cam_cy", camConfig_.cy);
    get_parameter("depth_scaling_factor", camConfig_.depth_scaling_factor);

    get_parameter("down_sample_factor", down_sample_factor_);
    get_parameter("depth_filter_tolerance", depth_filter_tolerance_);
    get_parameter("depth_filter_mindist", depth_filter_mindist_);
    get_parameter("depth_filter_margin", depth_filter_margin_);

    get_parameter("p_min", p_min_);
    get_parameter("p_max", p_max_);
    get_parameter("p_hit", p_hit_);
    get_parameter("p_mis", p_mis_);
    get_parameter("p_occ", p_occ_);
    get_parameter("p_def", p_def_);

    get_parameter("resolution", resolution_);
    get_parameter("local_x", local_x_);
    get_parameter("local_y", local_y_);
    get_parameter("local_z", local_z_);
    get_parameter("inflate_size", inflate_size_);

    get_parameter("use_global_map", use_global_map_);
    get_parameter("use_mask", use_mask_);

    // setup gridmap
    if (use_global_map_) {
      // global map setup (not shown)
    } else {
      gridmap_.setup(resolution_, Eigen::Vector3d(local_x_,local_y_,local_z_),
                     camConfig_.range, false);
      gridmap_.setupP(p_min_,p_max_,p_hit_,p_mis_,p_occ_,p_def_);
    }

    // publishers
    gridmap_inflate_pub_ = create_publisher<OccMap3d>("gridmap_inflate", 1);

    // subscriptions via message_filters
    depth_sub_.subscribe(this, "depth", rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, "odom", rmw_qos_profile_default);

    sync_.reset(new ImageOdomSynchronizer(ImageOdomSyncPolicy(100), depth_sub_, odom_sub_));
    sync_->registerCallback(std::bind(&MappingNodelet::depth_odom_callback, this, _1, _2));

    if (use_mask_) {
      target_odom_sub_ = create_subscription<Odometry>(
        "target", 1,
        std::bind(&MappingNodelet::target_odom_callback, this, _1));
    }
  }

private:
  void depth_odom_callback(const Image::ConstSharedPtr depth_msg,
                           const Odometry::ConstSharedPtr odom_msg)
  {
    if (callback_lock_.test_and_set()) return;

    // convert depth
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    cv::Mat depth_img = cv_ptr->image;
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      depth_img.convertTo(depth_img, CV_16UC1, camConfig_.depth_scaling_factor);
    }

    // compute camera pose
    Eigen::Vector3d body_p{
      odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z};
    Eigen::Quaterniond body_q{
      odom_msg->pose.pose.orientation.w,
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z};
    Eigen::Vector3d cam_p = body_q * cam2body_p_ + body_p;
    Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);

    // collect points
    std::vector<Eigen::Vector3d> obs_pts;
    int nr = depth_img.rows, nc = depth_img.cols;
    for (int i=depth_filter_margin_; i<nr-depth_filter_margin_; i+=down_sample_factor_) {
      for (int j=depth_filter_margin_; j<nc-depth_filter_margin_; j+=down_sample_factor_) {
        uint16_t d = depth_img.at<uint16_t>(i,j);
        double z = d ? double(d)/camConfig_.depth_scaling_factor : camConfig_.range+0.5;
        if (std::isnan(z)||std::isinf(z)|| z<depth_filter_mindist_) continue;
        double y = (i-camConfig_.cy)*z/camConfig_.fy;
        double x = (j-camConfig_.cx)*z/camConfig_.fx;
        Eigen::Vector3d p = cam_q * Eigen::Vector3d(x,y,z) + cam_p;
        obs_pts.push_back(p);
      }
    }

    gridmap_.updateMap(cam_p, obs_pts);

    if (use_mask_) {
      std::lock_guard lock(target_mutex_);
      Eigen::Vector3d ld = target_odom_ - Eigen::Vector3d(0.5,0.5,1.0);
      Eigen::Vector3d ru = target_odom_ + Eigen::Vector3d(0.5,0.5,1.0);
      gridmap_.setFree(ld,ru);
    }

    OccMap3d msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    gridmap_.to_msg(msg);
    gridmap_inflate_pub_->publish(msg);

    callback_lock_.clear();
  }

  void target_odom_callback(const Odometry::ConstSharedPtr msg) {
    std::lock_guard lock(target_mutex_);
    target_odom_.x() = msg->pose.pose.position.x;
    target_odom_.y() = msg->pose.pose.position.y;
    target_odom_.z() = msg->pose.pose.position.z;
  }

  // members
  CamConfig camConfig_;
  int down_sample_factor_, depth_filter_margin_;
  double depth_filter_tolerance_, depth_filter_mindist_;
  int p_min_,p_max_,p_hit_,p_mis_,p_occ_,p_def_;
  double resolution_;
  int local_x_, local_y_, local_z_, inflate_size_;
  bool use_global_map_, use_mask_;

  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;

  OccGridMap gridmap_;

  rclcpp::Publisher<OccMap3d>::SharedPtr gridmap_inflate_pub_;

  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<Odometry> odom_sub_;
  std::shared_ptr<ImageOdomSynchronizer> sync_;

  std::atomic_flag callback_lock_ = ATOMIC_FLAG_INIT;

  rclcpp::Subscription<Odometry>::SharedPtr target_odom_sub_;
  std::mutex target_mutex_;
  Eigen::Vector3d target_odom_;
};

}  // namespace mapping

RCLCPP_COMPONENTS_REGISTER_NODE(mapping::MappingNodelet)
