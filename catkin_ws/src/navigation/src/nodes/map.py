#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

def create_map():
    # Define map properties
    width = 32
    height = 32
    resolution = 0.3  # meters per cell

    # Create an occupancy grid
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info.resolution = resolution
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height

    # Define origin position in meters
    origin_x = -6 * resolution - resolution / 2
    origin_y = -17 * resolution - resolution / 2

    occupancy_grid.info.origin.position.x = origin_x
    occupancy_grid.info.origin.position.y = origin_y
    occupancy_grid.info.origin.position.z = 0.0
    occupancy_grid.info.origin.orientation.w = 1.0

    # Initialize map data (0: free, 100: obstacle, -1: unknown)
    grid_data = np.zeros((height, width), dtype=np.int8)

    # Obstacles in grid coordinates (x, y)
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

    for obstacle in obstacles:
        try:
            grid_data[obstacle[1]+17, obstacle[0]+6] = 100
        except IndexError:
            pass

    # Flatten the grid data to a list
    occupancy_grid.data = grid_data.flatten().tolist()

    return occupancy_grid

def publish_map():
    rospy.init_node('map')
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

    # Create the occupancy grid map
    occupancy_grid = create_map()

    # Set up the static transform broadcaster
    br = tf2_ros.StaticTransformBroadcaster()
    static_transform_stamped = TransformStamped()

    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"
    static_transform_stamped.child_frame_id = "odom"

    static_transform_stamped.transform.translation.x = 0.0
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0
    static_transform_stamped.transform.rotation.x = 0.0
    static_transform_stamped.transform.rotation.y = 0.0
    static_transform_stamped.transform.rotation.z = 0.0
    static_transform_stamped.transform.rotation.w = 1.0

    br.sendTransform(static_transform_stamped)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        occupancy_grid.header.stamp = rospy.Time.now()
        map_pub.publish(occupancy_grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_map()
    except rospy.ROSInterruptException:
        pass
