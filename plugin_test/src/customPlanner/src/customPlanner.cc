#include "customPlanner/customPlanner.hh"
#include <iostream>
#include <vector>
#include "astar_srv/srv/astar.hpp"
#include <chrono>
#include <concepts>

using namespace std::chrono_literals;

namespace customPlanner {
    void Planner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) {
        node_ = parent.lock();
        costmap_ = costmap_ros->getCostmap();

        // processing the costmap
        height_ = costmap_->getSizeInCellsX();
        width_ = costmap_->getSizeInCellsY();

        // RCLCPP_INFO(node_->get_logger(), "customPlanner data in X: %d", costmap_->getSizeInCellsX());
        // RCLCPP_INFO(node_->get_logger(), "customPlanner data in Y: %d", costmap_->getSizeInCellsY());
        // RCLCPP_INFO(node_->get_logger(), "customPlanner resolution: %f", costmap_->getResolution());
        // global_frame_ = costmap_ros->getGlobalFrameID(); // map
        // name_ = costmap_ros->getName(); // global_costmap
        
        // create a client for the server
        client = node_->create_client<astar_srv::srv::Astar>("astar_planner");

     }

    void Planner::activate() {
        RCLCPP_INFO(node_->get_logger(), "in activate state");
    }

    void Planner::deactivate() {
        RCLCPP_INFO(node_->get_logger(), "in deactivate state");
    }

    void Planner::cleanup() {
      RCLCPP_INFO(node_->get_logger(), "in cleanup state");
    }

    auto Planner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) -> nav_msgs::msg::Path 
    {
        nav_msgs::msg::Path path;

        double initial_value = 0;
        std::vector<double> costs(width_ * height_, initial_value);

        for (int row = 0; row < height_; row++) {
            for (int column = 0; column < width_; column++) {
                costs[(row * height_) + column] = costmap_->getCost(row, column);
            }
        }

        auto request = std::make_shared<astar_srv::srv::Astar::Request>();
        request->origin_x = costmap_->getOriginX();
        request->origin_y = costmap_->getOriginY();
        request->costmap = costs;
        request->width = width_;
        request->height = height_;
        request->resolution = costmap_->getResolution();

        while(!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }

        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "received response from service");
        } 
        else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service astar_planner");
        }

        /*
            TODO:
            *** Create a service in ros2 and call AStar/djikstra algorithm
        */
       
        RCLCPP_INFO(node_->get_logger(), "in createPlan");

        return path;

    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(customPlanner::Planner, nav2_core::GlobalPlanner)