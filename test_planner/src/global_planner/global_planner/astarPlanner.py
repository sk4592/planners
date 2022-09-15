#! /usr/bin/env python3

from math import atan2, floor, pi, cos, sin, tan, atan, sqrt
from turtle import distance
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from visualization_msgs.msg import MarkerArray, Marker
import copy
import threading
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
from global_planner.bezierPlanner import Bezier
import numpy as np


class Visualizer:
    def __init__(self, node: Node):
        self.node = node
        self.viz = self.node.create_publisher(MarkerArray, '/PathViz', 10)
        self.id = -1

    def createArrowsFrompath(self, data: list[tuple]):
        msg: MarkerArray = MarkerArray()
        for pose in data:
            marker = self.createArrow(pose)
            msg.markers.append(marker)
        
        return msg

    def createArrow(self, data: tuple):
        self.node.get_logger().info(f'data: {data}')
        rotation = R.from_euler('z', data[2], degrees=False)
        quat = rotation.as_quat()

        self.id += 1
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.header.stamp = self.node.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = self.id

        # Set the scale of the marker
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = float(data[0])
        marker.pose.position.y = float(data[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        return marker

@dataclass
class State:
    x: float
    y: float
    theta: float

    def getTuple(self):
        return (self.x, self.y, self.theta)

@dataclass
class StateWithPath:
    state: State
    path: list[State]

@dataclass
class StateWithHeuristics:
    x: float
    y: float
    theta: float
    g: float
    f: float

    def getState(self):
        return State(self.x, self.y, self.theta)


class Astar():
    def __init__(self, node, start, goal, costmap: OccupancyGrid, width, height) -> None:    
        self.node: Node = node
        self.visualizer = Visualizer(self.node)
        self.start: Point = start.position
        self.startOrient = start.orientation
        self.goal: Point = goal.position
        # self.goalOrient = goal.orientation
        self.costmap: OccupancyGrid = costmap
        self.width: int = width
        self.height: int = height
        self.map: list[list[int]] = []
        self.timer = self.node.create_timer(0.1, self.timer_callback)
        self.ogm_pub = self.node.create_publisher(OccupancyGrid, "/customMap", 10)
        self.mapOrigin: list[float] = []
        self.worldOrigin: list[int] = [0, 0]
        self.resolution: float
        self.msg = OccupancyGrid()
        self.count = 0
        self.path = None
        self.viz_msg = MarkerArray()
        self.angles = [0, pi/2]
        self.positions = [(0.5, 0.5), (0.5, -0.5), (0.5, 0.0)]

        rotate = R.from_quat([self.startOrient.x, self.startOrient.y, self.startOrient.z, self.startOrient.w])
        self.orient = rotate.as_euler('zxy', degrees=False)

    def timer_callback(self):
        
        self.msg.header = self.costmap.header
        self.msg.info = self.costmap.info
        self.msg.data = self.costmap.data
        self.resolution = self.costmap.info.resolution

        tmpOrigin = self.costmap.info.origin.position
        self.mapOrigin = [int(tmpOrigin.x), int(tmpOrigin.y)]

        # startcoord = self.indexFromWorldToMap(self.start)
        if (self.count == 0):
            t1 = threading.Thread(target=self.demoFunction)
            t1.start()
        self.count += 1

        

        # checking the initial point
        # pointInd = []
        # pointInd = self.indexFromWorldToMap(self.start)
        # index = self.getCoords(pointInd[1], pointInd[0])
        # msg.data[index] = 100

        # for neighborIndex in self.findNeighbors(pointInd):
        #     index = self.getCoords(neighborIndex[1], neighborIndex[0])
        #     msg.data[index] = 100

        # # checking the goal point
        # pointInd = []
        # pointInd = self.indexFromWorldToMap(self.goal)
        # index = self.getCoords(pointInd[1], pointInd[0])
        # msg.data[index] = 100

        # for neighborIndex in self.findNeighbors(pointInd):
        #     collision, index = self.collisionCheck(neighborIndex, msg.data)
        #     self.node.get_logger().info(f'collision {collision}')
        #     if not collision:
        #         msg.data[index] = 100
        
        self.ogm_pub.publish(self.msg)
        self.visualizer.viz.publish(self.viz_msg)

    def demoFunction(self):
        ptsList = []
        startcoord = self.indexFromWorldToMap(self.start)
        startind = self.getCoords(startcoord[1], startcoord[0])
        finalStates = self.expandNeighborsBezier(StateWithHeuristics(startcoord[0], startcoord[1], self.orient[0], 0.0, 0.0))
        for f in finalStates:
            f.path.append(f.state)
        for finalState in finalStates:
            for point in finalState.path:
                pts = self.indexFromMapToWorld(list((point.x, point.y)))
                self.node.get_logger().info(f"pts: {pts}")
                ptsList.append((pts[0], pts[1], point.theta))
        ptsList.append((self.start.x, self.start.y, self.orient[0]))
        self.viz_msg = self.visualizer.createArrowsFrompath(ptsList)

    def getPrimitives(self):
        primitives: list[State] = [
            # rightside of the point/state
            State(5, 5, 0.0),
            State(5, 5, pi / 2),

            # left side of the point/state
            State(5, -5, 0.0),
            State(5, -5, -pi / 2),

            # straiight to the robot
            State(5, 0, 0.0)
        ]
        distance = sqrt(5 ** 2 + 5 ** 2)
        return primitives, distance


    def bezierFunction(self, initialState: State):
        finalState: State = State(0.0, 0.0, 0.0) # initialize to fill
        finalStates: list[StateWithPath] = []
        primitives, distance = self.getPrimitives()


        for primitive in primitives:
            d = sqrt(primitive.x ** 2 + primitive.y ** 2)
            theta: float = 0.0
            if primitive.x != 0:
                theta = atan(primitive.y / primitive.x)
            theta += initialState.theta
            finalState.x = initialState.x + d * cos(theta)
            finalState.y = initialState.y + d * sin(theta)
            
            finalState.theta = initialState.theta + primitive.theta
            # print(f'finalState: {finalState}')
            # finalStates.append(copy.deepcopy(finalState))
            
            # self.finalState = State(self.initialState.x + pose[0], self.initialState.y + pose[1], self.initialState.theta + angle*y)
    
            self.cp1, self.cp2 = self.getControlPoints(initialState, finalState, distance)

            # states = [self.initialState, self.finalState, self.cp1, self.cp2]
            # for state in states:
            #     plt.plot(state.y, state.x, 'o')
            
            finalStates.append(copy.deepcopy(self.bezierEqn(initialState, finalState, self.cp1, self.cp2)))
            # print(finalStates)
        return finalStates

    def getControlPoints(self, initialState: State, finalState: State, dist: float):
        distance = dist
        cpx = initialState.x + (distance * (1/3)) * cos(initialState.theta)
        cpy = initialState.y + (distance * (1/3)) * sin(initialState.theta)
        cp1 = State(cpx, cpy, initialState.theta)

        cpx = finalState.x - (distance * (1/3)) * cos(finalState.theta)
        cpy = finalState.y - (distance * (1/3)) * sin(finalState.theta)
        cp2 = State(cpx, cpy, finalState.theta)

        return cp1, cp2

    def bezierEqn(self, initialState: State, finalState: State, cp1: State, cp2: State):
        pathStates: list[State] = []
        theta: float = 0
        for t in np.linspace(0,1,25):
            x: float = ((1 - t)**3) * (initialState.x) + 3 * ((1 - t)**2) * t * cp1.x + 3*(1-t)*(t**2)*cp2.x + (t**3) * finalState.x
            y: float = ((1 - t)**3) * (initialState.y) + 3 * ((1 - t)**2) * t * cp1.y + 3*(1-t)*(t**2)*cp2.y + (t**3) * finalState.y

            vecX = 3*(1-t**2)*(cp1.x - initialState.x) + 6*(1-t)*t*(cp2.x - cp1.x) + 3*(t**2)*(finalState.x - cp2.x)
            vecY = 3*(1-t**2)*(cp1.y - initialState.y) + 6*(1-t)*t*(cp2.y - cp1.y) + 3*(t**2)*(finalState.y - cp2.y)

            angle: float = atan2(vecY, vecX)
            theta += initialState.theta + angle

            pathStates.append(State(x, y, theta))
        
        return StateWithPath(finalState, pathStates) 

    def expandNeighborsBezier(self, state: StateWithHeuristics):
        neighbors: list[State] = []
        initialState: StateWithHeuristics = state

        finalStates: list[StateWithPath] = self.bezierFunction(initialState)
        return finalStates

    def collisionCheckBezier(self, state: StateWithPath, map: list[int]):
        states: list[State] = state.path
        states.append(state.state)
        for _state in states:
            ind = self.getCoords(int(_state.y), int(_state.x))
            if map[ind] >= 50:
                return True

        return False

    def runBezierAstar(self):
            startcoord = self.indexFromWorldToMap(self.start)
            startind = self.getCoords(startcoord[1], startcoord[0])

            goalcoord = self.indexFromWorldToMap(self.goal)
            goalind = self.getCoords(goalcoord[1], goalcoord[0])

            self.node.get_logger().info(f"startcoord: {startcoord}")
            self.node.get_logger().info(f"goalcoord: {goalcoord}")

            numberOfStacks = 90

            closedList = [[[0 for row in range(self.width)] for column in range(self.height)] for stack in range(numberOfStacks)]

            goalState: StateWithHeuristics = StateWithHeuristics(goalcoord[0], goalcoord[1], 0.0, 0.0, 0.0)

            pathData: dict[tuple, list] = {}
            startState: StateWithHeuristics = StateWithHeuristics(startcoord[0], startcoord[1], self.orient[0], 0.0, 0.0)
            stacknum = self.thetaToStack(startState.theta, numberOfStacks)
            closedList[stacknum][floor(startState.x)][floor(startState.y)] = 1
            openList: list[StateWithHeuristics] = []
            openList.append(startState)

            while len(openList) > 0:
                openList.sort(key=self.hybridSort)

                currNode: StateWithHeuristics = openList.pop(0)
                pathData[currNode.getState().getTuple()] = []

                if (int(currNode.x) == int(goalState.x)) and (int(currNode.y) == int(goalState.y)):
                    pathData[currNode.getState().getTuple()].append(goalState.getState().getTuple())
                    # path = self.constructPathHybrid(pathData, startState.getState().getTuple(), goalState.getState().getTuple())
                    # self.viz_msg = self.visualizer.createArrowsFrompath(path)
                    self.node.get_logger().info('path found')
                    # self.paint(path)
                    # self.node.get_logger().info(f'pathData: {pathData}')
                    return 


                for neighbor in self.expandNeighborsBezier(currNode):
                    # self.node.get_logger().info(f'neighbor: {neighbor}')
                    if self.collisionCheckBezier(neighbor, self.msg.data):
                        continue
                    
                
                    neighStack = self.thetaToStack(neighbor.theta, numberOfStacks)


                    neighborWithHeuristic: StateWithHeuristics = StateWithHeuristics(neighbor.state.x, neighbor.state.y, neighbor.state.theta, 0.0, 0.0)
                    neighborWithHeuristic.g += currNode.g + 0.0
                    neighborWithHeuristic.f += neighborWithHeuristic.g + self.heuristic(neighbor, goalState)

                    if closedList[neighStack][floor(neighbor.state.x)][floor(neighbor.state.y)] == 0:
                        pathData[currNode.getState().getTuple()].append(neighbor.state.getTuple())
                        closedList[neighStack][floor(neighbor.state.x)][floor(neighbor.state.y)] = 1
                        openList.append(neighborWithHeuristic)
                        # self.paintCell(neighbor)

                pass

            self.node.get_logger().info('No path found')




    def constructPathHybrid(self, path: dict[tuple, list[tuple]], start: tuple, goal: tuple):
        # self.node.get_logger().info(f'path: {path}')
        # self.node.get_logger().info(f'start: {start}')

        # self.node.get_logger().info(f'no of start: {list(path.keys()).count(start)}')

        def getKey(value: tuple):
            for key, values in path.items():
                if value in values:
                    return key

        constructedPath = []
        node = goal
        constructedPath.append(node)
        # node[0] == start[0]
        # node[1] == start[1]
        
        while True:
            if node[0] == start[0] and node[1] == start[1] and node[2] == start[2]:
                break

            key = getKey(node)
            # points = list((key[0], key[1]))
            points = self.indexFromMapToWorld(list((key[0], key[1])))
            constructedPath.append((points[0], points[1], key[2]))
            node = key
        
        # constructedPath.append(start)
        return constructedPath

    def heuristic(self, neighbor: State, goalState: State):
        return abs(neighbor.x - goalState.x) + abs(neighbor.y - goalState.y)

    def expandNeighbors(self, state: StateWithHeuristics):
        neighbors: list[State] = []
        initialDelta = -10
        finalDelta = 10
        stepSize = 2

        for delta in range(initialDelta, finalDelta, stepSize):
            neighState = self.bicycleModel(state.getState(), delta)
            neighbors.append(neighState)

        return neighbors  

    def bicycleModel(self, state: State, delta: float):
        SPEED = 0.9
        LENGTH = 0.5
        delta_ = delta * (pi / 180.0)
        omega_ = tan(delta_) * SPEED / LENGTH 
        theta_ = state.theta + omega_
        if theta_ < 0:
            theta_ += 2 * pi
        
        x_ = state.x + SPEED * cos(theta_)
        y_ = state.y + SPEED * sin(theta_)

        return State(x_, y_, theta_)

    def hybridSort(self, data: StateWithHeuristics):
        return round(data.f + data.g)

    def thetaToStack(self, theta: float, numberOfStacks: float):
        theta_ = (theta + 2 * pi) % (2 * pi)
        return int(round(theta_ * numberOfStacks) / (2 * pi)) % numberOfStacks

    def runHybridAstar(self):
        startcoord = self.indexFromWorldToMap(self.start)
        startind = self.getCoords(startcoord[1], startcoord[0])

        goalcoord = self.indexFromWorldToMap(self.goal)
        goalind = self.getCoords(goalcoord[1], goalcoord[0])

        self.node.get_logger().info(f"startcoord: {startcoord}")
        self.node.get_logger().info(f"goalcoord: {goalcoord}")

        numberOfStacks = 90

        closedList = [[[0 for row in range(self.width)] for column in range(self.height)] for stack in range(numberOfStacks)]

        goalState: StateWithHeuristics = StateWithHeuristics(goalcoord[0], goalcoord[1], 0.0, 0.0, 0.0)

        pathData: dict[tuple, list] = {}
        startState: StateWithHeuristics = StateWithHeuristics(startcoord[0], startcoord[1], self.orient[0], 0.0, 0.0)
        stacknum = self.thetaToStack(startState.theta, numberOfStacks)
        closedList[stacknum][floor(startState.x)][floor(startState.y)] = 1
        openList: list[StateWithHeuristics] = []
        openList.append(startState)

        while len(openList) > 0:
            openList.sort(key=self.hybridSort)

            currNode: StateWithHeuristics = openList.pop(0)
            pathData[currNode.getState().getTuple()] = []

            if (int(currNode.x) == int(goalState.x)) and (int(currNode.y) == int(goalState.y)):
                pathData[currNode.getState().getTuple()].append(goalState.getState().getTuple())
                path = self.constructPathHybrid(pathData, startState.getState().getTuple(), goalState.getState().getTuple())
                self.viz_msg = self.visualizer.createArrowsFrompath(path)
                self.node.get_logger().info('path found')
                # self.paint(path)
                # self.node.get_logger().info(f'pathData: {pathData}')
                return 


            for neighbor in self.expandNeighbors(currNode):
                # self.node.get_logger().info(f'neighbor: {neighbor}')
                if self.collisionCheck(list((neighbor.x, neighbor.y)), self.msg.data):
                    continue
                
            
                neighStack = self.thetaToStack(neighbor.theta, numberOfStacks)


                neighborWithHeuristic: StateWithHeuristics = StateWithHeuristics(neighbor.x, neighbor.y, neighbor.theta, 0.0, 0.0)
                neighborWithHeuristic.g += currNode.g + 0.0
                neighborWithHeuristic.f += neighborWithHeuristic.g + self.heuristic(neighbor, goalState)

                if closedList[neighStack][floor(neighbor.x)][floor(neighbor.y)] == 0:
                    pathData[currNode.getState().getTuple()].append(neighbor.getTuple())
                    closedList[neighStack][floor(neighbor.x)][floor(neighbor.y)] = 1
                    openList.append(neighborWithHeuristic)
                    # self.paintCell(neighbor)

            pass

        self.node.get_logger().info('No path found')
        






    def constructPath(self, path: dict[tuple, int], start: list[int], goal: list[int]):
        # self.node.get_logger().info(f'path: {path}')
        # self.node.get_logger().info(f'start: {start}')

        # self.node.get_logger().info(f'no of start: {list(path.keys()).count(start)}')

        def getKey(value: list[int]):
            for key, values in path.items():
                if list(value) in values:
                    return key

        constructedPath: list[list[int]] = []
        node = list(goal)
        start = list(start)
        constructedPath.append(node)
        # node[0] == start[0]
        # node[1] == start[1]
        
        while True:
            if node[0] == start[0] and node[1] == start[1]:
                break
            key = getKey(node)
            
            constructedPath.append(key)
            node = key
        
        # constructedPath.append(start)
        return constructedPath

    def cost(self, key: float):
        return key[1] # cost

    def run(self):
        costmap: list[int] = copy.deepcopy(self.costmap.data)

        startcoord = self.indexFromWorldToMap(self.start)
        startind = self.getCoords(startcoord[1], startcoord[0])

        goalcoord = self.indexFromWorldToMap(self.goal)
        goalind = self.getCoords(goalcoord[1], goalcoord[0])

        self.node.get_logger().info(f"startcoord: {startcoord}")
        self.node.get_logger().info(f"goalcoord: {goalcoord}")

        openList: list[tuple(list[int, int], float)] = []
        closedList: list[tuple(list[int, int], float)] = []

        pathData: dict[tuple, list] = {}

        openList.append((startcoord, 0))
        
        while len(openList) > 0:
            openList.sort(key=self.cost)
            
            frontNode = openList.pop(0)
            currNode = frontNode[0]

            pathData[tuple(currNode)] = []

            for neighbor, neighCost in self.findNeighbors(currNode):

                if self.collisionCheck(neighbor, self.msg.data):
                    continue

                if (neighbor[0] == goalcoord[0]) and (neighbor[1] == goalcoord[1]):
                    pathData[tuple(currNode)].append(neighbor)
                    path = self.constructPath(pathData, startcoord, goalcoord)
                    self.node.get_logger().info("path found")
                    # self.node.get_logger().info(f'path: {path}')
                    self.paint(path)
                    
                    return 

                else:
                    closedNodes = []
                    for closed in closedList:
                        closedNodes.append(list(closed))

                    if neighbor in closedNodes:
                        continue

                    stepCost = 0
                    f = 0 # abs(startcoord[0] - neighbor[0]) + abs(startcoord[1] - neighbor[1])
                    g = abs(goalcoord[0] - neighbor[0]) + abs(goalcoord[1] - neighbor[1])
                    h = stepCost + f + g

                    openNodes = []
                    for nodes in openList:
                        openNodes.append(nodes[0])
                    
                    if neighbor in openNodes:
                        # cmdInd = openNodes.index(neighbor)
                        # if openList[cmdInd][1] > h:
                        #     openList[cmdInd] = (openList[cmdInd][0], h)
                        continue
                    
                    openList.append((neighbor, h))
                    pathData[tuple(currNode)].append(neighbor)

            closedList.append(currNode)
            # self.paint(closedList)
        
    def paint(self, data: list[tuple[int]]):
        for cell in data:
            ind = self.getCoords(int(cell[1]), int(cell[0]))
            self.msg.data[ind] = 103

    def paintCell(self, data: State):
        ind = self.getCoords(int(data.y), int(data.x))
        self.msg.data[ind] = 103

    def collisionCheck(self, point: list[int, int], map: list[int]):
        index = self.getCoords(int(point[1]), int(point[0]))
        collision = False
        if map[index] >= 50:
            collision = True

        return collision

    # add collision check
    def findNeighbors(self, point: list[int, int]):
        neighbors = []
        
        neighbors.append(([point[0] + 1, point[1] + 0], 1))
        neighbors.append(([point[0] + 0, point[1] + 1], 1))
        neighbors.append(([point[0] + -1, point[1] + 0], 1))
        neighbors.append(([point[0] + 0, point[1] + -1], 1))

        neighbors.append(([point[0] + 1, point[1] + 1], 1.414))
        neighbors.append(([point[0] + 1, point[1] + -1], 1.414))
        neighbors.append(([point[0] + -1, point[1] + 1], 1.414))
        neighbors.append(([point[0] + -1, point[1] + -1], 1.414))
        
        return neighbors

    def vectorTransform(self, a: list[int], b: list[int]):
        return b[0] - a[0], b[1] - a[1]

    def indexFromWorldToMap(self, start: Point):
        # mapOrigin = [self.mapOrigin[0] / self.resolution, self.mapOrigin[1] / self.resolution]
        mTw = self.vectorTransform(self.mapOrigin, self.worldOrigin)
        wTp = self.vectorTransform(self.worldOrigin, [start.x, start.y])
        mTp = mTw[0] + wTp[0], mTw[1] + wTp[1]
        return int(mTp[0] / self.resolution), int(mTp[1] / self.resolution)

    def indexFromMapToWorld(self, point: list):
        wTm = self.vectorTransform(self.worldOrigin, self.mapOrigin)
        wTp = ((wTm[0]) + point[0]  * self.resolution), ((wTm[1]) + point[1]  * self.resolution)
        return wTp[0], wTp[1]

    def getCoords(self, row: int, column: int):
        return (row * self.width) + column

    def findPath(self) -> Path:
        closedList: Point = []
        openList: Point = []