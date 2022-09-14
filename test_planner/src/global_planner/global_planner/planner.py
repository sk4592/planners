#! /usr/bin/env python

from concurrent.futures import thread
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from global_planner.astarPlanner import Astar
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetMap
import threading

class Planner(Node):
    def __init__(self):
        super().__init__('global_planner')
        
        # subscriber for initial pose
        self.initialPose = PoseWithCovarianceStamped()
        self.initialPoseSub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.ipCallback, 10)

        # subscriber for goal pose
        self.goalPose = PoseStamped()
        self.goalPoseSub = self.create_subscription(PoseStamped, '/goal_pose', self.gpCallback, 10)

        # get map from map server and its meta data
        self.getMap = self.create_client(GetMap, '/map_server/map')
        self.req = GetMap.Request()
        self.getMap.wait_for_service(timeout_sec=1)
        self.res: GetMap.Response = self.send_request()

        self.goalPoseInit: bool = False
        self.initialPoseInit: bool = False

        data: list[int] = []
        # self.get_logger().info(f"getting back the result: {len(res.map.data)}")
        # self.get_logger().info(f"getting back the result: {res.map.info}")

        # data = list(filter(lambda x: x != -1, res.map.data))
        # self.get_logger().info(f"printing the known data: {data}")

    def plannerRun(self):
        while not self.initialPoseInit or not self.goalPoseInit:
            self.get_logger().info(f"waiting for initial pose and goal pose {self.initialPoseInit} {self.goalPoseInit}")
            time.sleep(1)
            pass
    
        start = self.initialPose.pose.pose
        goal = self.goalPose.pose
        costmap = self.res.map
        width = self.res.map.info.width
        height = self.res.map.info.height
        map: list[list[int]] = []

        self.globalPlanner = Astar(self, start, goal, costmap, width, height)
        # self.globalPlanner.run()

    def ipCallback(self, msg: PoseWithCovarianceStamped):
        self.initialPoseInit = True
        self.initialPose = msg
        self.get_logger().info(f"intial pose is {self.initialPose}")

    def gpCallback(self, msg: PoseStamped):
        self.goalPoseInit = True
        self.goalPose = msg
        self.get_logger().info(f"goal pose is {self.goalPose}")

    def send_request(self):
        self.future = self.getMap.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    planner = Planner()
    plan = threading.Thread(target=planner.plannerRun)
    plan.start()
    rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()