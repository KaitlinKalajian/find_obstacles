import rclpy
import math
import numpy as np
import cv2
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class MyMap():

  def __init__(self, min_x, max_x, min_y, max_y, res):
    self.res = res
    self.min_x = min_x
    self.max_x = max_x
    self.min_y = min_y
    self.max_y = max_y
    
    cols= (max_x - min_x)/res
    rows= (max_y - min_y)/res
    
    print(cols, rows)
    
    self.map = np.zeros((math.ceil(rows), math.ceil(cols)))
    
  def cartesian2cell(self, x , y):
    x_m = (x-self.min_x)/self.res
    y_m = (y-self.min_y)/self.res
    cell = np.array([x_m, y_m])
    return cell
  
  def addObstacle(self, x, y):
    cell = self.cartesian2cell(x, y)
    self.map[math.floor(cell[0])][math.floor(cell[1])]= 1.0
    print(self.map)

  def showMap(self):
      #show np array as an image using cv2
      cv2.imshow("map", self.map)
      cv2.waitKey(1)
      
class MyController(Node):

  def __init__(self):
    super().__init__('my_controller')
    print('I am creating an instance of MyController')

    self.my_map = MyMap(-10.0, 10.0, -10.0, 10.0, 0.05)
    
    #initialize
    self.min_right = 1000;
    self.min_left = 1000;
    
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0

    self.odom_mag = Odometry()
    self.scan_mag = LaserScan()
    
#creating subscribers
    self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 0) 
    self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 0) 

#creating publishers
    self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 0)
    
#creating timer for control callback
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.control_callback)
    
  def control_callback(self):
  
#accel. speed. Increase for faster, decrease for slower
    kx = 0.5
    ky = 0.5
#disance parameter. Changing can affect how "wiggly" bot moves   
    d = 0.1
     
    xd = 7.0
    yd = 8.0
    
    ex = xd-self.x
    ey = yd-self.y
    
    u1=kx*ex
    u2=ky*ey
    
    v = math.cos(self.theta)*u1+math.sin(self.theta)*u2
    om = (-math.sin(self.theta)*u1+math.cos(self.theta)*u2)/d
    
    if v>0.5:
      v=0.5
    if v<-0.5:
      v=-0.5
    if om>1.0:
      om = 1.0
    if om<-1.0:
      om = -1.0
      
#alternative controller (reactive)
    om = 0.1*(self.min_left - self.min_right)
    v=0.2
    
    if self.min_left<0.4 or self.min_right<0.4:
      v = 0.0
      om = 0.3
    
    if v>0.2:
      v=0.2
    if v<-0.2:
      v=-0.2
    if om>1.0:
      om = 1.0
    if om<-1.0:
      om = -1.0
      
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = om
    self.cmd_vel_publisher.publish(msg)
    
    #print(self.scan_msg.ranges)
    
    
  def odom_callback(self, msg):
  
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    
    self.theta = math.atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz))
    #self.odom_msg = msg
    
  def scan_callback(self, msg):
  
    print(self.x, self.y, self.theta)
    
    count = 0
    self.min_right = msg.range_max;
    self.min_left = msg.range_max;
    for d_i in msg.ranges:
      if d_i <= msg.range_max:
        gamma_i = msg.angle_min+count*msg.angle_increment
        if gamma_i>0.0 and gamma_i<1.57: #left side between 0 and pi/2
          if d_i<self.min_left:
            self.min_left = d_i
        if gamma_i>4.71 and gamma_i<6.28: #right side between 3pi/2 and 2pi
          if d_i<self.min_right:
            self.min_right = d_i
            
        #robot frame
        DxR = d_i * math.cos(gamma_i)
        DyR = d_i * math.sin(gamma_i)
        
        #world frame
        DxW = self.x + math.cos(self.theta)*DxR - math.sin(self.theta)*DyR
        DyW = self.y + math.sin(self.theta)*DxR + math.cos(self.theta)*DyR
        
        #print(DxW, DyW)
        self.my_map.addObstacle(DxW, DyW)
      count=count+1
      self.my_map.showMap()
      
      
def main(args=None):
    
    rclpy.init(args=args)
    print('Hi from my_package.')
    
    my_controller=MyController()
    
    rclpy.spin(my_controller)
    
    my_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
