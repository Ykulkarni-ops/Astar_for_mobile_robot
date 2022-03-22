import numpy as np
import math as m
import heapq as hp
import cv2
import pygame

#get user input
x_start= int(input("Enter the x coordinate of the start:  "))
y_start= int(input("Enter the y coordinate of the start:  "))

start_orientation = int(input("Enter the Orientation at start (enter in multiples of 30 degreees and less that 360 degrees), :  "))
x_goal= int(input("Enter the x coordinate of the goal:  "))
y_goal= int(input("Enter the y coordinate of the goal:  "))
#y_goal = 199-y_goal
radius= int(input("Enter the radius of the robot:  "))
clearance= int(input("Enter the clearance of the robot: "))
step_size = int(input("Enter the step (1-10): "))
start = (x_start,y_start)
goal = (x_goal,y_goal)


# fucntion to approximate the node value

def approx(x):
    if round(x)+0.5 == x:
        y = x
    elif   round(x) < x:
        y = round(x)
    else:
        y = round(x)
    return y

#heuristic (eculidean distance)
def heuristic(a,b):
    x1, y1 = a[0], a[1]
    x2, y2 = b[0], b[1]

    distance = approx(m.sqrt((x2-x1)**2 + (y2-y1)**2))
    return distance


#action set
def move0(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(0)*step + x)
    yr = approx(np.cos(0)*step + y)
    new = (xr,yr)

    return new

def move30(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(30))*step + x)
    yr = approx(np.cos(np.deg2rad(30))*step + y)
    new = (xr,yr)

    return new

def move60(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(60))*step + x)
    yr = approx(np.cos(np.deg2rad(60))*step + y)
    new = (xr,yr)

    return new

def move90(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(90))*step + x)
    yr = approx(np.cos(np.deg2rad(90))*step + y)
    new = (xr,yr)

    return new

def move120(vertex, step):
    x  = vertex[0]
    y = vertex[1]
    xr = approx(np.cos(np.deg2rad(30))*step + x)
    yr = approx(np.cos(np.deg2rad(30))*step + y)
    new = (xr,yr)

    return new


# calulating all possible points 
map_points = []
for i in range(801):
    for j in range(501):
        map_points.append((i/2,(j/2)))
print(len(map_points))

#all possible points in obstacle space

obstacle_space = []

for pt in map_points:

    x = pt[0]
    y = pt[1]

    #circle shape for path traversal
    if (((x - 300)*(x - 300)) + ((y - 185)*(y - 185)) <= (40*40)):
        obstacle_space.append((x,y))


    #hexagon shape for path traversal
    if x <= 235 and x >= 165 and (74*x - 175*y + 8825 >=0) and (74*x + 175*y - 38425 <=0) and (74*x - 175*y - 3425 <=0) and (74*x + 175*y - 26175 >=0):
        obstacle_space.append((x,y))


    #quad shape for traversal
    if ((25*x - 79*y + 13715 >= 0) and (6*x - 7*y + 780 <= 0) and (85*x + 69*y - 15825 >= 0)) or ((85*x + 69*y -15825>=0) and (16*x + 5*y - 2180<=0) and (25*x - 79*y + 13715 >=0)):
        obstacle_space.append((x,y))

print(len(obstacle_space))
