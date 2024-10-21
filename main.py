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

# Define start and goal positions
start = {'x': 1, 'y': 2}
goal = {'x': 6, 'y': 2}

# Define grid dimensions and obstacles
rows, cols = (10, 10)  # Update cols to 15 based on obstacle coordinates
obs = [
    {'x': 3, 'y': 2},
    {'x': 4, 'y': 2},
]

# Initialize grid
grid = [[0 for _ in range(cols)] for _ in range(rows)]  

# Pretty print the grid with path
def pretty_print_grid(grid, path):
    # Create a copy of the grid to modify
    grid_with_path = [row[:] for row in grid]
    
    # Mark the path positions with '*'
    for pos in path:
        grid_with_path[pos['y']][pos['x']] = '*'
    
    # Print the grid, showing the path as stars
    for row in grid_with_path:
        print(" ".join(f"{str(num):>3}" for num in row))

# Call pretty_print_grid with the path to visualize the path


for i in range(0, rows):
    for j in range(0, cols):
        distance = abs(goal['x'] - j) + abs(goal['y'] - i)
        grid[i][j] = distance

# Insert obstacles into the grid
def insert_obs():
    for o in obs:
        grid[o['y']][o['x']] = obsConstant

insert_obs()

# Manhattan distance function
def manhattan_distance(p1, p2):
    return abs(p1['x'] - p2['x']) + abs(p1['y'] - p2['y'])

# Get valid neighboring positions (up, down, left, right)
def get_neighbors(pos):
    neighbors = [
        {'x': pos['x'] + 1, 'y': pos['y']},  # right
        {'x': pos['x'] - 1, 'y': pos['y']},  # left
        {'x': pos['x'], 'y': pos['y'] + 1},  # down
        {'x': pos['x'], 'y': pos['y'] - 1}   # up
    ]
    # Filter out neighbors that are out of bounds
    return [n for n in neighbors if 0 <= n['x'] < cols and 0 <= n['y'] < rows]

def bfs_move_to_goal(start, goal):
    queue = deque([start])
    visited = set()
    visited.add((start['x'], start['y']))
    parent = {}  # To reconstruct the path

    while queue:
        current = queue.popleft()
        
        if current['x'] == goal['x'] and current['y'] == goal['y']:
            # Reconstruct path
            path = []
            while (current['x'], current['y']) != (start['x'], start['y']):
                path.append(current)
                current = parent[(current['x'], current['y'])]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor in get_neighbors(current):
            if grid[neighbor['y']][neighbor['x']] == obsConstant:
                continue  # Skip obstacles
            if (neighbor['x'], neighbor['y']) in visited:
                continue  # Skip visited
            queue.append(neighbor)
            visited.add((neighbor['x'], neighbor['y']))
            parent[(neighbor['x'], neighbor['y'])] = current  # Track path

    print("No path to goal!")
    return []

# Execute the pathfinding
path = bfs_move_to_goal(start, goal)
#grid[2][5] = -1
print(path)
pretty_print_grid(grid, path)
#pprint(path) 

#def clear_screen():
#    os.system('clear')  # For Linux and macOS, use 'cls' if on Windows

# Function to pretty print the grid and mark the current position with 'x'
#def pretty_print_grid_with_robot(grid, robot_pos):
#    for i, row in enumerate(grid):
#        for j, num in enumerate(row):
#            if i == robot_pos['y'] and j == robot_pos['x']:
#                print("  x", end=" ")  # Mark current position with 'x'
#            else:
#                print(f"{num:3d}", end=" ")
#        print()
# Move forward by one unit (no turns, just forward motion)
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
