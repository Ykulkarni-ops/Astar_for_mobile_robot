import cv2
import numpy as np
import math
from heapq import heappush, heappop, heapify




# create class for Astar
class Astar(object):

    #initialize the variables for start, goalm clearance, radius and position
    def __init__(self, start, goal, clearance, radius):
        self.start = start
        self.goal = goal
        self.numRows = 200
        self.numCols = 300
        self.i = 0
        self.j = 0
        self.clearance = clearance
        self.radius = radius

    
    # check if the clearance for the point robot is valid    
    def validClearance(self, currRow, currCol):
        return (currRow >= (1 + self.radius + self.clearance) and currRow <= (self.numRows - self.radius - self.clearance) and currCol >= (1 + self.radius + self.clearance) and currCol <= (self.numCols - self.radius - self.clearance))