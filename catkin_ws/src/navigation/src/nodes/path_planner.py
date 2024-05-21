#!/usr/bin/env python
'''
This node is for state-level control
Subscribed topics:
    odom                  [nav_msgs/Odometry]
    goal                  [geometry_msgs/Point]
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
obstacles = [(6, 1), (6, 0), (6, -1),       # ID1
             (7, 1), (7, 0), (7, -1),       # ID2
             (12, 3), (13, 3), (14, 3),     # ID3
             (12, 4), (13, 4), (14, 4),     # ID4
             (19, -1), (19, 0), (19, 1),    # ID5
             (20, -1), (20, 0), (20, 1),    # ID6 
             (25, 3), (26, 3), (27, 3),     # ID7
             (25, 4), (26, 4), (27, 4),     # ID8
             (-1, 3), (0, 3), (1, 3),       # ID11
             (-1, 4), (0, 4), (1, 4),       # ID12
             (7, -11), (7, -10), (7, -9),   # ID16
             (9, -9), (10, -9), (11, -9),    # ID18
             # teaching desk
             (8, -11), (8, -10), (8, -9),
             (9, -11), (9, -10), (9, -9),
             (10, -11), (10, -10), (10, -9),
             (11, -11), (11, -10), (11, -9),
             (12, -11), (12, -10), (12, -9)  # ID19
            ] 

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
        # self.goal_sub = rospy.Subscriber('clicked_point', PointStamped, self.goal_callback)
        self.goal_sub = rospy.Subscriber('goal', Point, self.goal_callback)

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
            self.start = (self.current_x, self.current_y)
            self.plan_path()

        # Publish the path
        self.path_pub.header.stamp = rospy.Time.now()
        self.path_pub.header.frame_id = 'map'
        self.path_pub.poses = [PoseStamped(pose=Pose(position=Point(x=x, y=y))) for x, y in self.path]
        self.pos_pub.publish(self.path_pub)

    def plan_path(self):
        start = self.start
        goals = self.goals

        # Plan the path
        goals_grid = [] # grid coorinate set
        goals_blank = [] # blank coodinate set

        for goal in goals:
            goal_grid = trans_phy_to_grid(goal[0], goal[1])
            goal_blank = trans_phy_to_blank(goal[0], goal[1])
            goals_grid.append(goal_grid) # input for astar
            goals_blank.append(goal_blank) # input for astar

        start_trans = trans_phy_to_blank(start[0], start[1])
        sta_goal = [start_trans] + goals_blank

        grid = create_grid(n, m, obstacles)
        grid_path = []
        phy_path = []
        all_grid_path = []
        all_phy_path = []
        # print("start & goal :", sta_goal)
        # print("# in sta_goal :", len(sta_goal))

        for i in range(len(sta_goal)-1):
            path = a_star_search(grid, sta_goal[i], sta_goal[i+1])
            for (blank_x, blank_y) in path:
                grid_path.append(trans_blank_to_grid(blank_x, blank_y))
                phy_path.append(trans_blank_to_phy(blank_x, blank_y))
            all_grid_path.append(grid_path)
            all_phy_path.append(phy_path)

        self.path = [item for sublist in all_phy_path for item in sublist]
        # rospy.loginfo(self.path)

    def goal_callback(self, data:Point):
        # Extract the goal from the clicked point only if the frame is "map"
        # return
        # if data.header.frame_id == 'map':
        #     self.goals = [(data.point.x, data.point.y)]
        self.goals = [(data.x, data.y)]

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
        grid_x, grid_y = trans_grid_to_blank(ox, oy)
        if 0 <= grid_x < len(grid) and 0 <= grid_y < len(grid[0]):
            grid[grid_x][grid_y] = 1
    return grid

# phy----refers to the physical coordinates in real world
# grid---refers to the grid coordinates that the origin is where the robot start
# blank--refers to the blank grid origin that the origin is located at the most left-bottom corner
############################################################################################
def trans_phy_to_blank(phy_x, phy_y):
    # Translate phy to grid first
    # Then translate grid to blank:
    grid_x, grid_y = trans_phy_to_grid(phy_x, phy_y)
    blank_x, blank_y = trans_grid_to_blank(grid_x, grid_y)
    return (blank_x, blank_y)

def trans_blank_to_phy(blank_x, blank_y):
    # Translate blank to grid first
    # Then translate grid to phy:
    grid_x, grid_y = trans_blank_to_grid(blank_x, blank_y)
    phy_x, phy_y = trans_grid_to_phy(grid_x, grid_y)
    return (phy_x, phy_y)

############################################################################################
def trans_blank_to_grid(blank_x, blank_y):
    # Translate blank coordinates into grid indices
    grid_x = blank_x - 6
    grid_y = 17 - blank_y
    return (grid_x, grid_y)

def trans_grid_to_blank(grid_x, grid_y):
    # Translate grid coordinates into blank indices
    blank_x = grid_x + 6
    blank_y = 17 - grid_y
    return (blank_x, blank_y)

############################################################################################
def trans_grid_to_phy(grid_x, grid_y, unit_size=0.3):
    # Convert grid coordinates to physical coordinates
    phy_x = grid_x * unit_size
    phy_y = grid_y * unit_size
    return (phy_x, phy_y)

def trans_phy_to_grid(phy_x, phy_y, unit_size=0.3):
    # Convert physical coordinates to grid coordinates
    grid_x = int(phy_x / unit_size)
    grid_y = int(phy_y / unit_size)
    return (grid_x, grid_y)

############################################################################################

if __name__ == '__main__':
    controller = PathPlanner()
    rospy.spin()
