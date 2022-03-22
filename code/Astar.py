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