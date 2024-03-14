#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np

class SimpleFrontierExplorer(Node):
    def __init__(self):
        super().__init__('simple_frontier_explorer')
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.costmap_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_timer = self.create_timer(10, self.set_next_goal)
        self.robot_position = None
        self.costmap = None
        self.free_threshold = 20  # Change this value to adjust the threshold for considering a cell as free

    def odom_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def costmap_callback(self, msg):
        self.costmap = msg

    def set_next_goal(self):
        if self.costmap is None:
            self.get_logger().info('No costmap received yet')
            return
        # Process costmap data to find frontiers
        frontiers = self.find_frontiers(self.costmap)
        self.get_logger().info('Number of frontiers found: %d' % len(frontiers))
        if frontiers:
            target_frontier = self.select_target_frontier(frontiers)
            goal_pose = self.frontier_to_goal_pose(target_frontier)
            self.goal_publisher.publish(goal_pose)

    def check_if_border_unknown(self, x, y, unknown_mask):
        return unknown_mask[y-1:y+2, x-1:x+2].any()
    
    def is_free(self, x, y, data):
        return data[y, x] <= self.free_threshold

    def find_frontiers(self, occupancy_grid):
        # Convert the flat array into a 2D numpy array for easier processing
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data).reshape((height, width))
        self.visited = set()

        # Define the state values in the occupancy grid
        UNKNOWN = -1
        
        # Create a mask for unknown cells
        unknown_mask = (data == UNKNOWN)
        
        # Initialize an empty list to hold frontiers
        frontiers = []
        
        # Iterate through the grid to find frontier cells
        for y in range(1, height - 1):  # Avoid the edges where we can't check all neighbors
            for x in range(1, width - 1):
                
                # Skip unknown cells
                if data[y, x] == UNKNOWN:
                    continue

                # Skip visited cells
                if (x, y) in self.visited:
                    continue
           
                # Check if the current cell is adjacent to an unknown cell
                if self.check_if_border_unknown(x, y, unknown_mask) and self.is_free(x, y, data):   
                    # If it is, find the connected component of frontier cells using a flood fill algorithm
                    frontier = self.flood_fill(data, (x, y), unknown_mask, UNKNOWN, width, height)
                    if frontier:
                        frontiers.append(frontier)
        
        return frontiers

    def flood_fill(self, data, start, unknown_mask, unknown_value, width, height):
        """
        A simple flood fill algorithm that finds and returns a list of connected cells with the target value
        starting from the 'start' cell. Only horizontal and vertical neighbors are considered.
        """
        x, y = start
        # Check if the start cell is within bounds
        if not (0 <= x < width and 0 <= y < height): # or data[y, x] != unknown_value:
            return []
        
        # Initialize a list to store the frontier cells and a set for visited cells
        frontier = []
        queue = [(x, y)]
        self.visited.add((x, y))
        
        while queue:
            x, y = queue.pop(0)
            
            # Add the current cell to the frontier if it's adjacent to an unknown cell
            frontier.append((x, y, data[y, x]))
            
            # Check and add the four neighbors (up, down, left, right) if they match the target value
            for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                nx, ny = x + dx, y + dy
                if data[y, x] == unknown_value:
                    continue
                if (0 <= nx < width and 0 <= ny < height and self.check_if_border_unknown(nx, ny, unknown_mask) and ((nx, ny) not in self.visited) and self.is_free(nx, ny, data)):
                    queue.append((nx, ny))
                    self.visited.add((nx, ny))
        
        return frontier


    def select_target_frontier(self, frontiers):
        if self.robot_position is None:
            self.get_logger().info('No robot position received yet')
            return None
        if not frontiers:
            self.get_logger().info('No frontiers found')
            return None
        
        # Assuming you have the robot's current position stored
        # This could be updated in the odom_callback method
        robot_x, robot_y = self.robot_position
        
        # Calculate the size of each frontier and its distance from the robot
        frontier_info = []  # List to hold tuples of (frontier index, size, distance)
        for index, frontier in enumerate(frontiers):
            size = len(frontier)  # The size is simply the number of cells in the frontier
            
            # Calculate the centroid of the frontier
            centroid_x = sum(x for x, y, cost in frontier) / size
            centroid_y = sum(y for x, y, cost in frontier) / size
            centroid_cost = sum(cost for x, y, cost in frontier) / size
            
            # Calculate the distance from the robot to the centroid of the frontier
            distance = ((centroid_x - robot_x) ** 2 + (centroid_y - robot_y) ** 2) ** 0.5
            
            frontier_info.append((index, size, centroid_cost, distance))
        
        # Now select the frontier that is the most free
        # Sort by size (descending) and then by distance (ascending)
        frontier_info.sort(key=lambda x: (x[2]))
        
        # The best frontier is the first in the sorted list
        best_frontier_index = frontier_info[0][0]
        best_frontier = frontiers[best_frontier_index]
        
        return best_frontier

    
    def frontier_to_goal_pose(self, frontier):
        # Calculate the centroid of the frontier
        centroid_x = sum(x for x, y, cost in frontier) / len(frontier)
        centroid_y = sum(y for x, y, cost in frontier) / len(frontier)
        
        # Create a PoseStamped message for the goal pose
        goal_pose = PoseStamped()
        
        # Fill in the header
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"  # Assuming the map frame; adjust if different
        
        # Set the position of the goal pose to the centroid
        goal_pose.pose.position.x = centroid_x * self.costmap.info.resolution + self.costmap.info.origin.position.x
        goal_pose.pose.position.y = centroid_y * self.costmap.info.resolution + self.costmap.info.origin.position.x
        goal_pose.pose.position.z = 0.0  # Assuming a flat ground
        
        # For simplicity, we'll set the orientation to face upwards
        # You might want to adjust this based on your navigation needs or robot orientation
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info('New goal pose: %s' % goal_pose.pose.position)
        
        return goal_pose


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
