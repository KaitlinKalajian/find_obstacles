import rclpy
import math
import numpy as np
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time


class MyMap:
    def __init__(self, min_x, max_x, min_y, max_y, res):
        self.res = res
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

        cols = (max_x - min_x) / res
        rows = (max_y - min_y) / res

        self.map = np.zeros((math.ceil(rows), math.ceil(cols)))
        self.obstacles=[]
        
    def cartesian2cell(self, x, y):
        x_m = (x - self.min_x) / self.res
        y_m = (y - self.min_y) / self.res
        cell = np.array([x_m, y_m])
        return cell

    def addObstacle(self, x, y):
        cell = self.cartesian2cell(x, y)
        self.map[math.floor(cell[0])][math.floor(cell[1])] = 1.0
        print(self.map)
        
    def showMap(self):
        # Show np array as an image using cv2
        cv2.imshow("map", self.map)
        cv2.waitKey(1)
        
    def get_obstacles(self):
        return self.obstacles  # Return the list of obstacles

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        print('I am creating an instance of MyController')

        self.my_map = MyMap(-10.0, 10.0, -10.0, 10.0, 0.05)

        # Initialize
        self.min_right = float('inf')
        self.min_left = float('inf')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_mag = Odometry()
        self.scan_mag = LaserScan()

        # Square exploration
        self.square_size = 1.0  # Initial square bounds (±1 unit)
        self.segment_start = (0.0, 0.0)  # Start of current segment
        self.segment_index = 0  # Index of current segment

        self.closest_obstacle = None

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 0)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 0)
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 0)

        # Timer for control callback
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_callback)

    def control_callback(self):
        # Execute the expanding square trajectory
        if self.closest_obstacle:
            # Move towards the closest obstacle if detected
            self.to_obstacle()
        else:
            # No obstacle detected, move in an expanding square trajectory
            self.expand_square_trajectory()

    def to_obstacle(self):
        if self.closest_obstacle:
            # Unpack all three values
            d_i, xd, yd = self.closest_obstacle

            ex = xd - self.x  # error in x
            ey = yd - self.y  # error in y

            d_obstacle = math.sqrt(ex**2 + ey**2)  # shortest distance to target

            # Proportional control to obstacle
            k_linear = 0.5
            k_angular = 2.0

            angle_to_goal = math.atan2(ey, ex)
            angle_error = angle_to_goal - self.theta

            # Normalize angle
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            msg = Twist()
            
            if d_obstacle <= 0.2:  # 20 cm threshold
                print(f"Obstacle reached")
                msg.linear.x = 0.0  # Stop moving forward
                msg.angular.z = 0.5  # Turn away
                self.cmd_vel_publisher.publish(msg)
                
                # Find the next closest obstacle
                self.closest_obstacle = self.find_next_obstacle()
            else:
                # Move towards the obstacle
                msg.linear.x = min(k_linear * d_obstacle, 0.5)
                msg.angular.z = k_angular * angle_error
                
            # Publish velocity
            self.cmd_vel_publisher.publish(msg)
        
    def find_next_obstacle(self):
        # Scan through all detected obstacles and return the closest one
        self.closest_obstacle = None
        for d_i, DxW, DyW in self.my_map.get_obstacles():  # Assuming `get_obstacles()` is implemented to return all obstacles
            if d_i < 2.0:  # Only consider obstacles within 2 meters
                if self.closest_obstacle is None or d_i < self.closest_obstacle[0]:
                    self.closest_obstacle = (d_i, DxW, DyW)
        return self.closest_obstacle

    def odom_callback(self, msg):
        # Update robot's position and orientation from odometry data
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def scan_callback(self, msg):
        print(self.x, self.y, self.theta)

        self.closest_obstacle = None
        count = 0.0
        self.min_right = msg.range_max
        self.min_left = msg.range_max

        # Laser scan angle of attack
        for d_i in msg.ranges:
            if d_i <= msg.range_max:
                gamma_i = msg.angle_min + count * msg.angle_increment
                if 0.0 < gamma_i < math.pi / 2:  # left side
                    if d_i < self.min_left:
                        self.min_left = d_i
                if 3 * math.pi / 2 < gamma_i < 2 * math.pi:  # right side
                    if d_i < self.min_right:
                        self.min_right = d_i

                # Robot frame
                DxR = d_i * math.cos(gamma_i)
                DyR = d_i * math.sin(gamma_i)

                # World frame
                DxW = self.x + math.cos(self.theta) * DxR - math.sin(self.theta) * DyR
                DyW = self.y + math.sin(self.theta) * DxR + math.cos(self.theta) * DyR

                # Add obstacles to the map
                self.my_map.addObstacle(DxW, DyW)

            count += 1
        self.my_map.showMap()

        # Track closest obstacle
        if self.closest_obstacle is None or d_i < self.closest_obstacle[0]:
            self.closest_obstacle = (d_i, DxW, DyW)

        # If an obstacle is detected and is within 2 meters, set it as the target
        if self.closest_obstacle and self.closest_obstacle[0] < 2.0:  # Obstacle within 2m
            self.obstacle_target = (self.closest_obstacle[1], self.closest_obstacle[2])
            print(f"Obstacle detected at: {self.obstacle_target}")

    def expand_square_trajectory(self):
        # No obstacle detected, move in an expanding square trajectory
        targets = [
            (self.square_size, 0),     # Move right
            (0, self.square_size),     # Move up
            (-self.square_size, 0),    # Move left
            (0, -self.square_size)     # Move down
        ]
        
        target = targets[self.segment_index]
        target_x, target_y = target
        
        # Move towards the next segment of the square path
        self.move_towards(target_x, target_y)
        
        # Check if the current segment is completed
        if abs(self.x - target_x) < 0.1 and abs(self.y - target_y) < 0.1:
            self.segment_index = (self.segment_index + 1) % 4
            if self.segment_index == 0:  # Completed a square
                self.square_size += 2.0  # Expand the square bounds
                print(f"Expanding square bounds to ±{self.square_size} units")

    def move_towards(self, target_x, target_y):
        # Simple method to move towards the target
        msg = Twist()
        msg.linear.x = 0.2  # Move at a constant speed
        self.cmd_vel_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    my_controller = MyController()
    rclpy.spin(my_controller)
    my_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

