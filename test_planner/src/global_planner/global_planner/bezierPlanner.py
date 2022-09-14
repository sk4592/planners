from math import atan, sqrt, tan, cos, sin, pi
from matplotlib import pyplot as plt
from dataclasses import dataclass
import numpy as np
import copy

@dataclass
class State:
    x: float
    y: float
    theta: float

    def getTuple(self):
        return (self.x, self.y, self.theta)

class Bezier:
    def __init__(self) -> None:
        self.speed = 0.9
        self.length = 0.05        

        self.x = 1.0
        self.y = 1.0

        self.angles = [0, pi/2]

        self.positions = [(1.0, 1.0), (1, -1.0), (1.0, 0.0)]
        
        self.initialState = State(0.0, 0.0, 0.0)
        self.finalState = State(0.0, 0.0, 0.0)

        finalStates = self.bezierFunction(self.initialState)
        print(finalStates)
        finalStates2 = []
        for nFinalState in finalStates:
            self.finalState = State(0.0, 0.0, 0.0)
            finalStates2.append(self.bezierFunction(nFinalState))

        for finals in finalStates2:
            for state in finals:
                self.finalState = State(0.0, 0.0, 0.0)
                self.bezierFunction(state)

        plt.grid()
        plt.show()

        # pass

    def bezierFunction(self, initialState: State):
        finalState: State = State(0.0, 0.0, 0.0) # initialize to fill
        finalStates: list[State] = []
        for pose in self.positions:
            for angle in self.angles:
                y = pose[1]
                d = sqrt(pose[0] ** 2 + pose[1] ** 2)
                theta: float = 0.0
                if pose[0] != 0:
                    theta = atan(pose[1] / pose[0])
                theta += initialState.theta
                finalState.x = initialState.x + d * cos(theta)
                finalState.y = initialState.y + d * sin(theta)
                finalState.theta = initialState.theta + angle * y
                # print(f'finalState: {finalState}')
                finalStates.append(copy.deepcopy(finalState))

                # self.finalState = State(self.initialState.x + pose[0], self.initialState.y + pose[1], self.initialState.theta + angle*y)
        
                self.cp1, self.cp2 = self.getControlPoints(initialState, finalState)

                # states = [self.initialState, self.finalState, self.cp1, self.cp2]
                # for state in states:
                #     plt.plot(state.y, state.x, 'o')
                
                xList, yList = self.bezierEqn(initialState, finalState, self.cp1, self.cp2)        
                for i in range(len(xList)):
                    plt.plot(yList[i], xList[i], 'o')
            # print(finalStates)
        return finalStates

    def getControlPoints(self, initialState: State, finalState: State):
        
        cpx = initialState.x + (1.414 * (1/3)) * cos(initialState.theta)
        cpy = initialState.y + (1.414 * (1/3)) * sin(initialState.theta)
        cp1 = State(cpx, cpy, initialState.theta)

        cpx = finalState.x - (1.414 * (1/3)) * cos(finalState.theta)
        cpy = finalState.y - (1.414 * (1/3)) * sin(finalState.theta)
        cp2 = State(cpx, cpy, finalState.theta)

        return cp1, cp2

    def bezierEqn(self, initialState, finalState, cp1, cp2):
        xList = []
        yList = []
        for t in  np.linspace(0,1,25):
            xList.append(((1 - t)**3) * (initialState.x) + 3 * ((1 - t)**2) * t * cp1.x + 3*(1-t)*(t**2)*cp2.x + (t**3) * finalState.x)
            yList.append(((1 - t)**3) * (initialState.y) + 3 * ((1 - t)**2) * t * cp1.y + 3*(1-t)*(t**2)*cp2.y + (t**3) * finalState.y)
        
        return xList, yList

if __name__ == "__main__":
    
    bezier = Bezier()
