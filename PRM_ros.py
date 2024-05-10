from LazyPRM_new import * # This imports the path data from the A_Star.py file
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class rosControlNode(Node):

    # Initialize Publisher
    def __init__(self):
        super().__init__('robot_control')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # Send commands to robot

    # We think there may be a conversion difference between truw time using the time.time() function compaored to the Gazebo simulation time that was
    # introducing an error to the commands
    
    def robotControl(self, path):
      i = 0
      t = 1.12 # time gain to fix difference between real time and sim time
      velocity_message = Twist()

      for self.action in path:
        self.lin = self.action[0]/100 # Linear velocity at each node converted to meters/s from centimeters/s
        self.ang = self.action[1] # angular velocity at each node in rad/s
        self.ts =  time.time() #start time
        self.tc = time.time() # current time

        velocity_message.linear.x = cl*float(self.lin) # m/s
        velocity_message.angular.z = ca*float(self.ang) # rad/s
        self.cmd_vel_pub.publish(velocity_message)

        # Print functions showing linear and angular velocity at each node, as well as a time stamp and step number
        print('printing  ', self.lin, '  and  ', self.ang, '\n')
        print(self.tc, '\n')
        print('finished step ', i, '\n')
        i = i + 1

        self.ts =  time.time() #start time
        self.tc = time.time()

        # Stop the robot at the end
        while self.tc - self.ts <= t:
        
          self.tc = time.time()
        
        velocity_message.linear.x = 0.0 # m/s
        velocity_message.angular.z = 0.0 # rad/s
        self.cmd_vel_pub.publish(velocity_message)
        
      print('FINISHED', '\n')

def main(args=None):
    rclpy.init(args=args)
    node = rosControlNode()
    node.robotControl(path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
