#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.closest_distance = float('inf')
        self.get_logger().info('Obstacle Avoidance Node Started')

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float32)

            height, width = depth_array.shape
            self.get_logger().info(f'Depth image shape: {depth_array.shape}')

            # Focus on the central part of the depth image for obstacle detection
            central_region = depth_array[height//3:2*height//3, width//3:2*width//3]
            
            # Filter out invalid values
            valid_depths = central_region[np.isfinite(central_region) & (central_region > 0)]
            
            if valid_depths.size > 0:
                self.closest_distance = np.min(valid_depths)
                self.get_logger().info(f'Closest distance in central region: {self.closest_distance:.2f} meters')
            else:
                self.closest_distance = float('inf')  # No valid depth values
                self.get_logger().info('No valid depth values in central region')

            # Visualization
            depth_visualization = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            cv2.imshow('Depth Visualization', depth_visualization)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Depth callback failed: {e}")

    def control_loop(self):
        twist = Twist()
        if self.closest_distance < 0.5:  # Very close to an obstacle
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn in place
            self.get_logger().info('Obstacle very close! Turning in place.')
        elif self.closest_distance < 0.8:  # Approaching an obstacle
            twist.linear.x = 0.1  # Slow down
            twist.angular.z = 0.2  # Slightly turn
            self.get_logger().info('Approaching obstacle! Slowing down and turning slightly.')
        else:  # No obstacle nearby
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info('No obstacle detected. Moving forward.')
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

