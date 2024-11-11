import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import heapq

# Define warehouse dimensions and settings
warehouse_width, warehouse_height = 10, 10
start = (0, 0)
destination = (7, 9)
robot_speed = 0.1  # meters per second
move_time = 0.1    # seconds to move 0.1m
stop_time = 2      # seconds to stop

# Define obstacles
obstacles = [(3, 3), (3, 4), (3, 5), (4, 3), (5, 3), (3, 7), (0, 5),(4,9),(1,5)]

# A* Pathfinding Algorithm
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar_pathfinding(start, goal, grid_size):
    grid = np.zeros((grid_size[0], grid_size[1]))
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        # Check if destination is reached
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        # Explore neighbors (up, down, left, right)
        neighbors = [(current[0] + 1, current[1]), (current[0] - 1, current[1]),
                     (current[0], current[1] + 1), (current[0], current[1] - 1)]
        
        for neighbor in neighbors:
            # Ensure neighbor is within bounds and not an obstacle
            if (0 <= neighbor[0] < grid_size[0] and 
                0 <= neighbor[1] < grid_size[1] and 
                neighbor not in obstacles):
                
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Simulate robot's movement with constraints
def simulate_robot_movement(path, grid_size):
    fig, ax = plt.subplots()
    ax.set_xlim(0, grid_size[0])
    ax.set_ylim(0, grid_size[1])
    ax.set_xticks(range(grid_size[0] + 1))
    ax.set_yticks(range(grid_size[1] + 1))
    plt.grid()
    plt.gca().invert_yaxis()
    
    # Mark start and destination
    ax.add_patch(patches.Circle((start[0] + 0.5, start[1] + 0.5), 0.2, color="DodgerBlue", label="Start"))
    ax.add_patch(patches.Circle((destination[0] + 0.5, destination[1] + 0.5), 0.2, color="Brown", label="Destination"))
    
    # Draw obstacles
    for obs in obstacles:
        ax.add_patch(patches.Rectangle((obs[0], obs[1]), 1, 1, color="gray"))
        ax.add_patch(patches.Rectangle((obs[0], obs[1]), 1, 1, color="gray"))
    ax.add_patch(patches.Rectangle((obs[0], obs[1]), 1, 1, color="gray", label="Obstacles"))

    # Create a basic robot shape with two wheels
    
    robot_body = patches.Rectangle((0.3, 0.75), 0.25, 0.25, color="#ebc738")
    robot_bbody = patches.Rectangle((0.3, 0.75), 0.4, 0.4, color="#f77760")
    # robot_bbody = patches.Circle((0.75, 0.75), 0.26, color="#eb5338")
    wheel_left = patches.Circle((0.15, 0.75), 0.04, color="black")
    wheel_right = patches.Circle((0.45, 0.75), 0.04, color="black")
    robot_left_leg = patches.Rectangle((0.3, 0.75), 0.05, 0.20, color="#141240")
    robot_right_leg = patches.Rectangle((0.3, 0.75), 0.05, 0.20, color="#141240")
        
    
    # Add robot parts to the plot
    
    ax.add_patch(robot_bbody)
    ax.add_patch(robot_body)
    ax.add_patch(wheel_left)
    ax.add_patch(wheel_right)
    ax.add_patch(robot_left_leg)
    ax.add_patch(robot_right_leg)
    
    # Initialize plot legend
    plt.legend(loc="upper right")

    # Animate the robot's movement
    for position in path:
        # Update robot parts' positions
        robot_left_leg.set_xy((position[0] + 0.42, position[1] + 0.8))
        robot_right_leg.set_xy((position[0] + 0.55, position[1] + 0.8))
        robot_body.set_xy((position[0] + 0.38, position[1] + 0.1))
        robot_bbody.set_xy((position[0] + 0.32, position[1] + 0.4))
        # robot_bbody.center = (position[0] + 0.32, position[1] + 0.60)
        wheel_left.center = (position[0] + 0.48, position[1] + 0.23)
        wheel_right.center = (position[0] + 0.63, position[1] + 0.23)
        
        plt.draw()
        plt.pause(move_time)  # Simulate movement time
        time.sleep(stop_time)  # Simulate stopping time

    plt.show()

# Run pathfinding and simulate robot's movement
path = astar_pathfinding(start, destination, (warehouse_width, warehouse_height))
if path:
    print("Path found:", path)
    simulate_robot_movement(path, (warehouse_width, warehouse_height))
else:
    print("No path found.")
