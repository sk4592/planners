#include "BTPlanner/Tree.hh"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

using SingleThreadedExecutor = rclcpp::executors::SingleThreadedExecutor;
using MultiThreadedExecutor = rclcpp::executors::MultiThreadedExecutor;

void spinRcl(std::shared_ptr<SingleThreadedExecutor> executor) {
    executor->spin();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    std::shared_ptr<Planner> planner = std::make_shared<Planner>("planner");
    factory.registerSimpleAction("initialPose", std::bind(&Planner::initialPose, planner));
    factory.registerSimpleAction("goalPose", std::bind(&Planner::goalPose, planner));
    factory.registerSimpleAction("waitMap", std::bind(&Planner::waitMap, planner));
    factory.registerSimpleAction("cleanup", std::bind(&Planner::shutdown, planner));
    
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("BTPlanner");

    auto tree = factory.createTreeFromFile(package_share_directory + "/treeXml/test.xml");
    std::unique_ptr<BT::Tree> treePtr = std::make_unique<BT::Tree>(std::move(tree));
    
    std::shared_ptr<SingleThreadedExecutor> executor = std::make_shared<SingleThreadedExecutor>();
    executor->add_node(planner);
    std::thread spinThread(std::bind(&spinRcl, executor));

    treePtr->tickRoot();
    spinThread.join();
    rclcpp::shutdown();

    return 0;
}