import numpy as np
import matplotlib.pyplot as plt
import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def is_empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def heuristic(a, b):
    # Using Chebyshev distance as the heuristic
    (x1, y1) = a
    (x2, y2) = b
    return max(abs(x1 - x2), abs(y1 - y2)) * 10  # Multiplied by 10 to keep consistent with step costs

def a_star_search(grid, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.is_empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next, cost in neighbors(grid, current):
            new_cost = cost_so_far[current] + cost
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return reconstruct_path(came_from, start, goal)

def neighbors(grid, node):
    directions = [(1, 0, 10), (-1, 0, 10), (0, 1, 10), (0, -1, 10),  # Orthogonal movements
                  (1, 1, 14), (-1, -1, 14), (1, -1, 14), (-1, 1, 14)]  # Diagonal movements
    (x, y) = node
    result = []
    for dx, dy, cost in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != 1:
            result.append(((nx, ny), cost))
    return result

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def create_grid(n, m, obstacles):
    grid = [[0] * m for _ in range(n)]
    for (ox, oy) in obstacles:
        grid_x, grid_y = translate_coords(ox, oy)
        if 0 <= grid_x < len(grid) and 0 <= grid_y < len(grid[0]):
            grid[grid_x][grid_y] = 1
    return grid

def translate_coords(x, y):
    # Translate grid coordinates from (-m, -w) to (n, o) into array indices
    grid_x = x + 6
    grid_y = y + 17
    return (grid_x, grid_y)

def translate_back(path):
    # Translate grid coordinates from (-m, -w) to (n, o) into array indices
    # Convert grid coordinates to physical coordinates
    return [(x - 6, y - 17) for x, y in path]

def translate_to_grid_coordinates(x, y, unit_size=0.3):
    # Convert physical coordinates to grid coordinates
    grid_x = int(x / unit_size)
    grid_y = int(y / unit_size)
    return (grid_x, grid_y)

def translate_to_physical_coordinates(path, unit_size=0.3):
    # Convert grid coordinates to physical coordinates
    return [(x * unit_size, y * unit_size) for x, y in path]


# def plot_grid(grid, start, goal, path):
#     fig, ax = plt.subplots(figsize=(32, 32))
#     ax.imshow(grid, cmap='gray_r', origin='lower')  # Ensure origin matches your coordinate system

#     # Update start and goal coordinates for plotting
#     start_plot = start
#     goal_plot = goal
#     path_plot = path

#     ax.scatter([p[1] for p in path_plot], [p[0] for p in path_plot], color='blue', label='Path')
#     ax.scatter(start_plot[1], start_plot[0], color='green', marker='o', s=100, label='Start')
#     ax.scatter(goal_plot[1], goal_plot[0], color='red', marker='X', s=100, label='Goal')
#     ax.scatter(6, 17, color='blue', label='Path')

#     # Annotate grid indices
#     for i in range(len(grid)):
#         for j in range(len(grid[0])):
#             ax.text(j, i, f'({i}, {j})', ha='center', va='center', color='gray', fontsize=6)

#     ax.set_title('Pathfinding with A*')
#     ax.legend()
#     plt.show()
def plot_grid(grid, start, goals, all_paths):
    fig, ax = plt.subplots(figsize=(12, 12))  # Adjust size as needed
    ax.imshow(grid, cmap='gray_r', origin='lower')  # Ensure origin matches your coordinate system
    
    # Mark the start and goal points
    ax.scatter(start[0]+17, start[1]+6, color='green', marker='o', s=100, label='Start')
    for goal in goals:
        gx, gy = translate_coords(*translate_to_grid_coordinates(goal[0], goal[1]))
        ax.scatter(gy, gx, color='red', marker='X', s=100, label=f'Goal {goals.index(goal)+1}')

    # Plot each path
    for path in all_paths:
        if path:
            y_coords, x_coords = zip(*[translate_coords(px, py) for px, py in path])  # Convert path to plot coordinates
            ax.plot(x_coords, y_coords, marker='o', linestyle='-', markersize=5, linewidth=2, label=f'Path to goal {all_paths.index(path)+1}')

    # Annotate grid indices (optional, can be removed for clarity)
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            ax.text(j, i, f'({i-6}, {j-17})', ha='center', va='center', color='gray', fontsize=6)
    
    ax.set_title('Pathfinding with A* for Multiple Goals')
    ax.legend()
    plt.show()

# map limit  
#            grid:   y: -17 ~ 14    ;   x: -6 ~ 25
#        physical:   y:5.25 ~ -4.35 ;   x: -1.95 ~ 7.65
#     grid Origin:  (0,0) -> (6,17)

n, m = 32, 32  # Dimensions of the grid
obstacles = [(6, 1), (6, 0), (6, -1), 
             (11, -2), (12, -2), (13, -2), 
             (19, -1), (19, 0), (19, 1), 
             (24, -2), (25, -2), (26, -2), 
             (25, -2), (26, -2), (27, -2), 
             (-1, -2), (0, -2), (1, -2), 
             (7, 11), (7, 10), (7, 9), 
             (8, 9), (9, 9), (10, 9),
             (8, 10), (9, 10), (10, 10),
             (8, 11), (9, 11), (10, 11),
            (11, 11), (11, 10), (11, 9)]  # Obstacles
start = (5,5)
goals = [(0, -3), (3, -3), (4, 2),(4, 0)]
goals_trans = []

for goal in goals:
    goal_grid = translate_to_grid_coordinates(goal[0], goal[1])
    goal_trans = translate_coords(goal_grid[0], goal_grid[1])
    goals_trans.append(goal_trans)

start_trans = translate_coords(start[0], start[1])
sta_goal = [start_trans] + goals_trans

grid = create_grid(n, m, obstacles)
all_block_path = []
all_physical_path = []
for i in range(len(sta_goal)-1):
    path = a_star_search(grid, sta_goal[i], sta_goal[i+1])
    block_path = translate_back(path)
    physical_path = translate_to_physical_coordinates(block_path)
    all_block_path.append(block_path)
    all_physical_path.append(physical_path)

path = [item for sublist in all_physical_path for item in sublist]

print(all_physical_path)
print(path)

# print("Block Path from", start, "to", goal_grid, ":", block_path)

# # Print the results
# for i, path in enumerate(all_block_path):
#     print(f"Block path coordinates to goal {i+1}(in meters): {path}")

# for i, path in enumerate(all_physical_path):
#     print(f"Physical path coordinates to goal {i+1}(in meters): {path}")

# plot_grid(grid, start, goals, all_block_path)

