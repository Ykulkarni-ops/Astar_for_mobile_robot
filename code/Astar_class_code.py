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


         # check triangles
        (x1, y1) = (120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r))
        (x2, y2) = (150 - sqrt_of_c_and_r, 50)
        (x3, y3) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist3 = 1
        if(first <= 0 and second <= 0 and third <= 0):
            dist3 = 0
            
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        (x3, y3) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist4 = 1
        if(first >= 0 and second >= 0 and third >= 0):
            dist4 = 0
        
        # check rhombus
        (x1, y1) = (10 - sqrt_of_c_and_r, 225)
        (x2, y2) = (25, 200 - sqrt_of_c_and_r)
        (x3, y3) = (40 + sqrt_of_c_and_r, 225)
        (x4, y4) = (25, 250 + sqrt_of_c_and_r)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist5 = 1
        dist6 = 1
        if(first >= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist5 = 0
            dist6 = 0
        
        # check square
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (120 - sqrt_of_c_and_r, 75)
        (x3, y3) = (150, 100 + sqrt_of_c_and_r)
        (x4, y4) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist7 = 1
        dist8 = 1
        if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
            dist7 = 0
            dist8 = 0
        
        # check rod
        first = ((col - 95) * (8.66 + sqrt_of_c_and_r)) - ((5 + sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        second = ((col - 95) * (37.5 + sqrt_of_c_and_r)) - ((-64.95 - sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        third = ((col - 30.05 + sqrt_of_c_and_r) * (8.65 + sqrt_of_c_and_r)) - ((5.45 + sqrt_of_c_and_r) * (row - 67.5))
        fourth = ((col - 35.5) * (-37.49 - sqrt_of_c_and_r)) - ((64.5 + sqrt_of_c_and_r) * (row - 76.15 - sqrt_of_c_and_r))
        dist9 = 1
        dist10 = 1
        if(first <= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist9 = 0
            dist10 = 0
        
        if(dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
            return True
        return False


    #action to move 30 degree up
    def ActionMove30Up(self, currRow, currCol, currangle):
        theta = 30 + currangle
        if(theta >= 360):
            theta = theta - 360
        rad = theta * (3.14/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)
        i1 = round(self.i,2)
        j2 = round(self.j)
        
        if(self.IsValid(currRow - self.i, currCol + self.j) and self.IsObstacle(currRow - self.i, currCol + self.j) == False):
            return True
        return False


    #action to move 60 degree up
    def ActionMove60Up(self, currRow,currCol, currangle):
        theta = 60 + currangle
        if(theta >= 360):
            theta = theta - 360
        rad = theta * (3.14/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)
        i1 = round(self.i)
        j2 = round(self.j,2)
        
        if(self.IsValid(currRow - self.i, currCol + self.j) and self.IsObstacle(currRow - self.i, currCol + self.j) == False):
            return True
        return False


    #action to move 30 degree down
    def ActionMove30Down(self, currRow,currCol, currangle):
        theta = currangle - 30
        if(theta < 0):
            theta = 360 + theta
        rad = theta * (3.14/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)
        i1 = round(self.i,2)
        j2 = round(self.j)
        
        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i, currCol + self.j) == False):
            return True
        return False


    #action to move 60 degree down
    def ActionMove60Down(self, currRow,currCol, currangle):
        theta = currangle - 60
        if(theta < 0):
            theta = 360 + theta
        rad = theta * (3.14/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)
        i1 = round(self.i)
        j2 = round(self.j,2)
        
        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i, currCol + self.j) == False):
            return True
        return False

    #action to move straight
    def ActionMoveStraight(self, currRow,currCol, currangle):
        theta = 0
        rad = theta * (3.14/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)
        i1 = round(self.i)
        j2 = round(self.j)
        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i , currCol + self.j) == False):
            return True
        return False

    #check if the goal is reached 
    def CheckIfGoal(self, currRow, currCol):
        check = (((currRow - self.goal[0]) * (currRow - self.goal[0])) + ((currCol - self.goal[1]) * (currCol - self.goal[1])) - ( 1.5 * 1.5))
        if(check <= 0):
            global cat
            global dog
            cat = currRow
            dog = currCol
            print("goal reached")
            return True
        else:
            return False


    # Astar algo

     # astar algorithm
    def Astar(self):
        # create hashmap to store distances
        distMap = {}
        visited = {}
        path = {}
        for row in np.arange(1, self.numRows + 1, 0.5):
            for col in np.arange(1, self.numCols + 1, 0.5):
                for angle in range(0,360,30):
                    distMap[(row, col, angle)] = float('inf')
                    path[(row, col, angle)] = -1
                    visited[(row, col, angle)] = False
            
        # create queue, push the source and mark distance from source to source as zero
        explored_states = []
        queue = []
        heappush(queue, (0, self.start))
        distMap[self.start] = 0

        while(len(queue) > 0):
            heapify(queue)
            _, currNode = heappop(queue)
            visited[currNode] = True
            explored_states.append(currNode)
        
            # if goal node then exit
            if(self.CheckIfGoal(currNode[0],currNode[1]) == True):
                break
           
            # go through each edge of current node
            a1 = currNode[2] + 30
            if(a1 >= 360):
                a1 = a1 - 360

            a2 = currNode[2] + 60
            if(a2 >= 360):
                a2 = a2 - 360

            a3 = currNode[2] - 30
            if(a3 < 0):
                a3 = 360 + a3

            a4 = currNode[2] - 60
            if(a4 < 0):
                a4 = 360 + a4


            h_dist = (((currNode[0] - self.goal[0]) ** 2) + 
                       ((currNode[1] - self.goal[1]) ** 2))
            h_dist = math.sqrt(h_dist)
            
        
            
            
            if(self.ActionMove30Up(currNode[0], currNode[1], currNode[2]) and visited[(currNode[0] + round(self.j), currNode[1] + round(self.i), a1)] == False and (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a1)] > (distMap[currNode] + h_dist+1.12))):
                distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a1)] = distMap[currNode] + h_dist +1.12
                path[(currNode[0] + round(self.j), currNode[1] + round(self.i), a1)] = currNode
                heappush(queue, (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a1)], (currNode[0] + round(self.j), currNode[1] + round(self.i), a1)))

            if(self.ActionMove60Up(currNode[0], currNode[1], currNode[2]) and visited[(currNode[0] + round(self.j), currNode[1] + round(self.i), a2)] == False and (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a2)] > distMap[currNode] + h_dist+1.12)):
                distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a2)] = distMap[currNode] + h_dist+1.12
                path[(currNode[0] + round(self.j), currNode[1] + round(self.i), a2)] = currNode
                heappush(queue, (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a2)], (currNode[0] + round(self.j), currNode[1] + round(self.i), a2)))

            if(self.ActionMove30Down(currNode[0], currNode[1], currNode[2]) and visited[(currNode[0] + round(self.j), currNode[1] + round(self.i), a3)] == False and (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a3)] > distMap[currNode] + h_dist+1.12)):
                distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a3)] = distMap[currNode] + h_dist+1.12
                path[(currNode[0] + round(self.j), currNode[1] + round(self.i), a3)] = currNode
                heappush(queue, (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a3)], (currNode[0] + round(self.j), currNode[1] + round(self.i), a3)))

            if(self.ActionMove60Down(currNode[0], currNode[1], currNode[2]) and visited[(currNode[0] + round(self.j), currNode[1] + round(self.i), a4)] == False and (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a4)] > distMap[currNode] + h_dist+1.12)):
                # print("60Down",a4)
                distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a4)] = distMap[currNode] + h_dist+1.12
                path[(currNode[0] + round(self.j), currNode[1] + round(self.i), a4)] = currNode
                heappush(queue, (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), a4)], (currNode[0] + round(self.j), currNode[1] + round(self.i), a4)))

            if(self.ActionMoveStraight(currNode[0], currNode[1], currNode[2]) and visited[(currNode[0] + round(self.j), currNode[1] + round(self.i), currNode[2])] == False and (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), currNode[2])] > distMap[currNode] + h_dist+1)):
                # print("Straights none")
                distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), currNode[2])] = distMap[currNode] + h_dist+1
                path[(currNode[0] + round(self.j), currNode[1] + round(self.i), currNode[2])] = currNode
                heappush(queue, (distMap[(currNode[0] + round(self.j), currNode[1] + round(self.i), currNode[2])], (currNode[0] + round(self.j), currNode[1] + round(self.i), currNode[2])))


        
