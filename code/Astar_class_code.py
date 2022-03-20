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


    #check the obstacle space and plot it 
    def obstacle(self, row, col):
        sum = self.clearance + self.radius
        sqrt_rc = 1.4142 * sum

        # check circle
        dist1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225)) - ((25 + sum) * (25 + sum))
        
        # check eclipse
        dist2 = ((((row - 100) * (row - 100)) / ((20 + sum) * (20 + sum))) + (((col - 150) * (col - 150)) / ((40 + sum) * (40 + sum)))) - 1