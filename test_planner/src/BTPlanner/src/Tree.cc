#include "BTPlanner/Tree.hh"

void Planner::ipCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& data) {
    RCLCPP_INFO(this->get_logger(), "subscribing to initial pose");
}