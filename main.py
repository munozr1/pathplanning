#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
#                                  InfraredSensor, UltrasonicSensor, GyroSensor)
# from pybricks.parameters import Port, Stop, Direction, Button, Color
# from pybricks.tools import wait, StopWatch, DataLog
# from pybricks.robotics import DriveBase
# from pybricks.media.ev3dev import SoundFile, ImageFile
from collections import deque


# ev3 = EV3Brick()
# left_motor = Motor(Port.B)
# right_motor = Motor(Port.C)
# speed = 200
# oneFoot = 505
# turn = 196

obsConstant = 99
start = {'x': 1, 'y': 1}
goal = {'x': 14, 'y': 8}

# Define grid dimensions and obstacles
rows, cols = (10, 16)  # Update cols to 15 based on obstacle coordinates
obs = [
   {'x':3,'y':1}, 
   {'x':3,'y':3}, 
   {'x':8,'y':4}, 
   {'x':8,'y':5}, 
   {'x':8,'y':6}, 
   {'x':8,'y':7}, 
]

# Initialize grid
grid = [[0 for _ in range(cols)] for _ in range(rows)]  

def pretty_print_grid(grid, path=None):
    # Create a copy of the grid to modify
    grid_with_path = [row[:] for row in grid]
    
    # If path is provided, mark the path positions with '*'
    if path:
        for pos in path:
            grid_with_path[pos['y']][pos['x']] = '*'  # Correctly mark with '*'
    
    # Print the grid, showing the path as stars (if applicable)
    for row in grid_with_path:
        print(" ".join(f"{str(num):>3}" for num in row))


for x in range(rows):
    for y in range(cols):
        distance = abs(goal['x'] - x) + abs(goal['y'] - y)
        grid[x][y] = distance

# Insert obstacles into the grid
def insert_obs():
    for o in obs:
        grid[o['y']][o['x']] = obsConstant  # Note: x and y are reversed here

insert_obs()

def manhattan_distance(p1, p2):
    return abs(p1['x'] - p2['x']) + abs(p1['y'] - p2['y'])

def get_neighbors(pos):
    neighbors = [
        {'x': pos['x'] + 1, 'y': pos['y']}, 
        {'x': pos['x'] - 1, 'y': pos['y']}, 
        {'x': pos['x'], 'y': pos['y'] + 1}, 
        {'x': pos['x'], 'y': pos['y'] - 1} 
    ]
    return [n for n in neighbors if 0 <= n['x'] < cols and 0 <= n['y'] < rows and grid[n['y']][n['x']] != obsConstant]

def bfs_move_to_goal(start, goal):
    queue = deque([start])
    visited = set()
    visited.add((start['x'], start['y']))
    parent = {} 


    while queue:
        current = queue.popleft()

        if current['x'] == goal['x'] and current['y'] == goal['y']:
            print("Valid Path found")
            path = []
            while (current['x'], current['y']) != (start['x'], start['y']):
                path.append(current)
                current = parent[(current['x'], current['y'])]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor in get_neighbors(current):
            if (neighbor['x'], neighbor['y']) in visited:
                continue  # Skip visited
            queue.append(neighbor)
            visited.add((neighbor['x'], neighbor['y']))
            parent[(neighbor['x'], neighbor['y'])] = current

    print("No path to goal found")
    return []

# Execute the pathfinding
path = bfs_move_to_goal(start, goal)
grid[start['y']][start['x']] = 'S'
grid[goal['y']][goal['x']] = 'G'
pretty_print_grid(grid, path)

def move_forward():
    print("move_forward")
    # left_motor.run_angle(speed, oneFoot, wait=False)
    # right_motor.run_angle(speed, oneFoot, wait=True) 
    # left_motor.stop()
    # right_motor.stop()

def turn_left_90():
    print("turn_left_90")
    # left_motor.run_angle(speed, -turn, wait=False) 
    # right_motor.run_angle(speed, turn, wait=True)   
    # left_motor.stop()
    # right_motor.stop()

def turn_right_90():
    print("turn_right_90")
    # left_motor.run_angle(speed, turn, wait=False)  
    # right_motor.run_angle(speed, -turn, wait=True)  
    # left_motor.stop()
    # right_motor.stop()

def move_left():
    # print("move_left")
    turn_left_90()
    move_forward()
    turn_right_90()  

# Function to move right (90-degree turn right, move forward, 90-degree turn back)
def move_right():
    # print("move_right")
    turn_right_90()  # Turn 90 degrees right
    move_forward()   # Move forward after turning right
    turn_left_90()   # Turn 90 degrees left to face forward again

# Move the robot following the path, using forward and turning logic
def move(path, grid):
    curr = start
    for move in path:
        xdiff = move['x'] - curr['x']
        ydiff = move['y'] - curr['y']

        while xdiff != 0 or ydiff != 0:
            # Move in the x direction (left/right)
            if xdiff > 0:
                move_right()  
                curr['x'] += 1
                xdiff -= 1
            elif xdiff < 0:
                move_left()
                curr['x'] -= 1
                xdiff += 1

            # Move in the y direction (forward/backward)
            if ydiff > 0:
                move_forward()
                curr['y'] += 1
                ydiff -= 1
            elif ydiff < 0:
                move_forward()
                curr['y'] -= 1
                ydiff += 1

move(path, grid)
