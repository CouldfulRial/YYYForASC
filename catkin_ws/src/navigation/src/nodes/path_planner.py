#!/usr/bin/env python
'''
This node is for state-level control
Subscribed topics:
    odom                  [nav_msgs/Odometry]
Published topics:
    path                  [nav_msgs/Path]
'''

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
import tf
import heapq

GRID_SIZE = 0.3  # m
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

class PathPlanner:
    def __init__(self):
        # Initialise node
        self.node_name = 'path_planner'
        rospy.init_node(self.node_name)

        # Get user parameters
        try:
            self.parm  = rospy.get_param(self.node_name)
            self.verbosity = self.parm["verbosity"]
        except KeyError:
            self.verbosity = 1

        # Subscribed topics
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        # For debug, subscribe to a point from clicked point in RVIZ
        self.goal_sub = rospy.Subscriber('clicked_point', PointStamped, self.goal_callback)

        # Timer: Update the path at 1Hz
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        # Published topics:
        self.pos_pub = rospy.Publisher('path', Path, queue_size=10)

        # Initialise parameters
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.start = (0, 0)
        self.goals = [(0, 0)]
        self.prev_goals = None
        self.path = []
        self.path_pub = Path()

    def timer_callback(self, event):
        # Start planning paths if there are new goals
        if (self.prev_goals is None) or (self.goals != self.prev_goals):
            self.prev_goals = self.goals
            rospy.loginfo(self.start)
            self.start = (  # Convert to grid
                round(self.current_x / GRID_SIZE),
                round(self.current_y / GRID_SIZE)
            )
            self.plan_path()

        # Publish the path
        self.path_pub.header.stamp = rospy.Time.now()
        self.path_pub.header.frame_id = 'map'
        self.path_pub.poses = [PoseStamped(pose=Pose(position=Point(x=x, y=y))) for x, y in self.path]
        self.pos_pub.publish(self.path_pub)

    def plan_path(self):
        # Plan the path
        goals_trans = []
        for goal in self.goals:
            goal_grid = translate_to_grid_coordinates(goal[0], goal[1])
            goal_trans = translate_coords(goal_grid[0], goal_grid[1])
            goals_trans.append(goal_trans)

        start_trans = translate_coords(self.start[0], self.start[1])
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

        self.path = [item for sublist in all_physical_path for item in sublist]
        rospy.loginfo(self.path)

    def goal_callback(self, data:PointStamped):
        # Extract the goal from the clicked point only if the frame is "map"
        if data.header.frame_id == 'map':
            self.goals = [(data.point.x, data.point.y)]

    def odom_callback(self, data:Odometry):
        # Extract the current pose from the odometry message
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        self.current_yaw = data.pose.pose.orientation
        self.current_yaw = self.quat_to_euler(self.current_yaw)

    ##############################################################################################################
    ############################## Untilities ####################################################################
    ##############################################################################################################

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

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

if __name__ == '__main__':
    controller = PathPlanner()
    rospy.spin()
