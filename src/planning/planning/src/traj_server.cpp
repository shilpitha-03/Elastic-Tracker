#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/poly_traj.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <traj_opt/poly_traj_utils.hpp>

using namespace std::chrono_literals;

class TrajServer : public rclcpp::Node {
public:
  TrajServer()
  : Node("traj_server")
  {
    // pubs & subs
    auto qos10 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    poly_traj_sub_ = this->create_subscription<quadrotor_msgs::msg::PolyTraj>(
      "trajectory", qos10,
      std::bind(&TrajServer::polyTrajCallback, this, _1));
    heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "heartbeat", qos10,
      std::bind(&TrajServer::heartbeatCallback, this, _1));
    pos_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "position_cmd", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    cmd_timer_ = this->create_wall_timer(
      10ms, std::bind(&TrajServer::cmdCallback, this));

    RCLCPP_WARN(this->get_logger(), "[Traj server]: ready.");
  }

private:
  void publish_cmd(int traj_id,
                   const Eigen::Vector3d &p,
                   const Eigen::Vector3d &v,
                   const Eigen::Vector3d &a,
                   double y, double yd)
  {
    quadrotor_msgs::msg::PositionCommand cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id;
    cmd.position.x = p.x(); cmd.position.y = p.y(); cmd.position.z = p.z();
    cmd.velocity.x = v.x(); cmd.velocity.y = v.y(); cmd.velocity.z = v.z();
    cmd.acceleration.x = a.x(); cmd.acceleration.y = a.y(); cmd.acceleration.z = a.z();
    cmd.yaw = y; cmd.yaw_dot = yd;
    pos_cmd_pub_->publish(cmd);
    last_p_ = p;
  }

  bool exe_traj(const quadrotor_msgs::msg::PolyTraj &trajMsg) {
    double t = (this->now() - trajMsg.start_time).seconds();
    if (t < 0) return false;
    if (trajMsg.hover) {
      Eigen::Vector3d p(trajMsg.hover_p[0], trajMsg.hover_p[1], trajMsg.hover_p[2]);
      publish_cmd(trajMsg.traj_id, p, {}, {}, last_yaw_, 0.0);
      return true;
    }
    // build Trajectory
    int pieces = trajMsg.duration.size();
    std::vector<double> dura(pieces);
    std::vector<CoefficientMat> cMats(pieces);
    for (int i = 0; i < pieces; ++i) {
      dura[i] = trajMsg.duration[i];
      int i6 = i * 6;
      for (int axis = 0; axis < 3; ++axis)
        for (int j = 0; j < 6; ++j)
          cMats[i].row(axis)(j) =
            axis==0 ? trajMsg.coef_x[i6+j] :
            axis==1 ? trajMsg.coef_y[i6+j] :
                      trajMsg.coef_z[i6+j];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      RCLCPP_ERROR(this->get_logger(), "trajectory too short left!");
      return false;
    }
    auto p = traj.getPos(t), v = traj.getVel(t), a = traj.getAcc(t);
    double yaw = trajMsg.yaw;
    // smooth yaw change
    double dy = yaw - last_yaw_;
    if (std::fabs(dy) > 0.02) yaw = last_yaw_ + (dy/std::fabs(dy))*0.02;
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, 0.0);
    last_yaw_ = yaw;
    return true;
  }

  void heartbeatCallback(const std_msgs::msg::Empty::SharedPtr) {
    heartbeat_time_ = this->now();
  }

  void polyTrajCallback(const quadrotor_msgs::msg::PolyTraj::SharedPtr msg) {
    trajMsg_ = *msg;
    if (!receive_traj_) {
      trajMsg_last_ = trajMsg_;
      receive_traj_ = true;
    }
  }

  void cmdCallback() {
    if (!receive_traj_) return;
    if ((this->now() - heartbeat_time_).seconds() > 0.5) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Lost heartbeat from planner");
      publish_cmd(trajMsg_.traj_id, last_p_, {}, {}, 0.0, 0.0);
      return;
    }
    if (exe_traj(trajMsg_)) {
      trajMsg_last_ = trajMsg_;
    } else {
      exe_traj(trajMsg_last_);
    }
  }

  // subs, pubs, timer
  rclcpp::Subscription<quadrotor_msgs::msg::PolyTraj>::SharedPtr poly_traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  // state
  quadrotor_msgs::msg::PolyTraj trajMsg_, trajMsg_last_;
  bool receive_traj_ = false;
  Eigen::Vector3d last_p_{0,0,0};
  double last_yaw_ = 0.0;
  rclcpp::Time heartbeat_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajServer>());
  rclcpp::shutdown();
  return 0;
}
