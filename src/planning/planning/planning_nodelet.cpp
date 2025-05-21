#include "rclcpp/rclcpp.hpp"
#include "quadrotor_msgs/Trajectory.h"

class PlanningNode : public rclcpp::Node
{
public:
    PlanningNode() : Node("planning_node")
    {
        RCLCPP_INFO(this->get_logger(), "Planning Node initialized.");
        traj_pub_ = this->create_publisher<quadrotor_msgs::msg::Trajectory>("trajectory", 10);
    }

private:
    rclcpp::Publisher<quadrotor_msgs::msg::Trajectory>::SharedPtr traj_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanningNode>());
    rclcpp::shutdown();
    return 0;
}
