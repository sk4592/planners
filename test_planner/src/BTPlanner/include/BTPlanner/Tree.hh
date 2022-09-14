#pragma once

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>

class Planner : public rclcpp::Node {
    public:
        Planner(std::string name) : Node (name) {
            // initial pose subscriber
            sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose", 
                10, 
                std::bind(&Planner::ipCallback, this, std::placeholders::_1)
            );
        }
        ~Planner() = default;

        BT::NodeStatus initialPose() {
            std::cout << "getting initial pose\n";
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus goalPose() {
            std::cout << "getting goal pose\n";
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus waitMap() {
            std::cout << "waiting for map\n";
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shutdown() {
            std::cout << "shutting down\n";
            return BT::NodeStatus::SUCCESS;
        }

    private:
        // subscriber for getting initial point
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
        void ipCallback(const geometry_msgs::msg::PoseWithCovarianceStamped&);
};