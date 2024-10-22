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
goal = {'x': 1, 'y': 1}
start = {'x': 14, 'y': 8}

# Define grid dimensions and obstacles
rows, cols = (10, 16)
obs = [
    {'x': 3, 'y': 1},
    {'x': 3, 'y': 3},
    {'x': 8, 'y': 4},
    {'x': 8, 'y': 5},
    {'x': 8, 'y': 6},
    {'x': 8, 'y': 7},
]

# Initialize grid
grid = [[0 for _ in range(cols)] for _ in range(rows)]

def pretty_print_grid(grid, path=None):
    grid_with_path = [row[:] for row in grid]
    if path:
        for pos in path:
            grid_with_path[pos['y']][pos['x']] = '*'
    
    for row in grid_with_path:
        print(" ".join(f"{str(num):>3}" for num in row))

for x in range(rows):
    for y in range(cols):
        distance = abs(goal['x'] - x) + abs(goal['y'] - y)
        grid[x][y] = distance

# Insert obstacles into the grid
def insert_obs():
    for o in obs:
        grid[o['y']][o['x']] = obsConstant

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
            path = []
            while (current['x'], current['y']) != (start['x'], start['y']):
                path.append(current)
                current = parent[(current['x'], current['y'])]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor in get_neighbors(current):
            if (neighbor['x'], neighbor['y']) in visited:
                continue
            queue.append(neighbor)
            visited.add((neighbor['x'], neighbor['y']))
            parent[(neighbor['x'], neighbor['y'])] = current

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

# New function to move the robot in reverse
def move_reverse():
    print("move_reverse")
    # left_motor.run_angle(speed, -oneFoot, wait=False)
    # right_motor.run_angle(speed, -oneFoot, wait=True) 
    # left_motor.stop()
    # right_motor.stop()

# Move the robot following the path, using forward and turning logic
def move(path, grid):
    curr = start
    direction = 'left'  # Start facing right

    for move in path:
        xdiff = move['x'] - curr['x']
        ydiff = move['y'] - curr['y']

        while xdiff != 0 or ydiff != 0:
            # Handle horizontal movement
            if xdiff > 0:  # Moving right
                if direction != 'right':
                    turn_right_90()
                    direction = 'right'
                move_forward()
                curr['x'] += 1
                xdiff -= 1
            elif xdiff < 0:  # Moving left
                if direction != 'left':
                    turn_left_90()
                    direction = 'left'
                move_forward()
                curr['x'] -= 1
                xdiff += 1

            # Handle vertical movement
            if ydiff > 0:  
                if direction != 'down':
                    turn_left_90()  
                    direction = 'down'
                move_forward()
                curr['y'] += 1
                ydiff -= 1
            elif ydiff < 0:  
                if direction != 'up':
                    turn_right_90()  
                    direction = 'up'
                move_forward()
                curr['y'] -= 1
                ydiff += 1

move(path, grid)
