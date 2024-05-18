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

    # Define obstacles (example: list of grid coordinates that are obstacles)
    obstacles = [(6, 1),  (6, 0),   (6, -1), 
                (11, -2), (12, -2), (13, -2), 
                (19, -1), (19, 0),  (19, 1), 
                (24, -2), (25, -2), (26, -2), 
                (25, -2), (26, -2), (27, -2), 
                (-1, -2), (0, -2),  (1, -2), 
                (7, 11),  (7, 10),  (7, 9), 
                (8, 9),   (9, 9),   (10, 9),
                (8, 10),  (9, 10),  (10, 10),
                (8, 11),  (9, 11),  (10, 11),
                (11, 11), (11, 10), (11, 9)]

    for obstacle in obstacles:
        try:
            grid_data[-obstacle[1]+17, obstacle[0]+6] = 100
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
