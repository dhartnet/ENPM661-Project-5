from LazyPRM import * # This imports the path data from the A_Star.py file
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
    # introducing an error to the commands. TO fix this we had to add some gains in places where the error accrued significantly
    
    def robotControl(self, optimizedPath):
      i = 0
      # Gains for linear and angular velocity and step time used throughout the code
      cl = 1.0 #0.995 # lin gain
      ca = 1.0 #0.9 # ang gain
      t = 1.12
      velocity_message = Twist()

      for item in optimizedPath:
        # Extract position, linear velocity and angulaw velocity from the path
        self.coord = item[0]
        self.x = self.coord[0] # x-coordinate of robot for positional awareness
        self.action = item[2]
        self.lin = self.action[0]/100 # Linear velocity at each node converted to meters/s from centimeters/s
        self.ang = -self.action[1] # angular velocity at each node in rad/s
        self.ts =  time.time() #start time
        self.tc = time.time() # current time


        if self.x > 475: # and self.x < 450:
          self.ang = self.ang * 0.2

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

#optimizedPath = [((50, 100, 0), 0, (0, 0), (50, 100, 0)), ((80, 100, 0), 30.0, (27.646015351590176, 0.0), (80.41061688674918, 100.0, 0.0)), ((102, 105, 1), 23.0, (20.734511513692635, 0.4816378981113271), (101.8937654988317, 105.37677754845187, 30.35540069686412)), ((115, 113, 1), 15.0, (13.823007675795088, 0.0), (115.16818338434045, 112.60265422168737, 29.999999999999996)), ((141, 128, 1), 30.0, (27.646015351590176, 0.0), (141.3363667686809, 128.2053084433746, 29.999999999999996)), ((163, 134, 0), 23.0, (20.734511513692635, -0.4816378981113271), (162.64894588071346, 134.29045680195867, 359.64459930313586)), ((193, 134, 0), 30.0, (27.646015351590176, 0.0), (193.41061688674918, 134.0, 0.0)), ((223, 134, 0), 30.0, (27.646015351590176, 0.0), (223.41061688674918, 134.0, 0.0)), ((253, 134, 0), 30.0, (27.646015351590176, 0.0), (253.41061688674918, 134.0, 0.0)), ((283, 134, 0), 30.0, (27.646015351590176, 0.0), (283.41061688674944, 134.0, 0.0)), ((313, 134, 0), 30.0, (27.646015351590176, 0.0), (313.4106168867495, 134.0, 0.0)), ((320, 136, 1), 8.0, (6.911503837897544, 0.4816378981113271), (320.29792183294387, 135.79225918281725, 30.35540069686412)), ((333, 144, 1), 15.0, (13.823007675795088, 0.0), (333.1681833843403, 143.60265422168737, 29.999999999999996)), ((359, 159, 1), 30.0, (27.646015351590176, 0.0), (359.3363667686806, 159.20530844337475, 29.999999999999996)), ((381, 165, 0), 23.0, (20.734511513692635, -0.4816378981113271), (380.6489458807135, 165.29045680195867, 359.64459930313586)), ((411, 165, 0), 30.0, (27.646015351590176, 0.0), (411.4106168867495, 165.0, 0.0)), ((441, 165, 0), 30.0, (27.646015351590176, 0.0), (441.4106168867495, 165.0, 0.0)), ((463, 160, 11), 23.0, (20.734511513692635, -0.4816378981113271), (462.8937654988317, 159.62322245154817, 329.64459930313586)), ((477, 159, 1), 15.0, (13.823007675795088, 0.9632757962226542), (477.49673330116013, 159.39134708656388, 30.71080139372816)), ((491, 160, 10), 15.0, (13.823007675795088, -0.9632757962226542), (491.49673330116013, 159.60865291343612, 329.2891986062718)), ((507, 144, 11), 23.0, (20.734511513692635, 0.4816378981113271), (506.60330869687306, 143.72783166773837, 330.3554006968641)), ((533, 129, 11), 30.0, (27.646015351590176, 0.0), (533.3363667686806, 128.79469155662525, 329.99999999999994)), ((559, 114, 11), 30.0, (27.646015351590176, 0.0), (559.3363667686806, 113.79469155662541, 329.99999999999994)), ((572, 106, 11), 15.0, (13.823007675795088, 0.0), (572.1681833843403, 106.39734577831263, 329.99999999999994))]


def main(args=None):
    rclpy.init(args=args)
    node = rosControlNode()
    node.robotControl(optimizedPath)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
