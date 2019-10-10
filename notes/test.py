#!/usr/bin/env python

# Controller for differential wheels mobile robots
# Author: Huy Pham - Email: huy.pham@eng.au.dk
# Note: change the way you input x_goal and y_goal, and play with the default parameters: 


import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, sin, cos

class MyRobot:

    def __init__(self):
        # Creates a node with name 'myrobot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('myrobot_controller', anonymous=True)

        # Publisher which will publish to the topic '/husky_velocity_controller/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/husky_velocity_controller/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.modelstate_subscriber = rospy.Subscriber('/gazebo/model_states',
                                                ModelStates, self.update_pose_info)

        # self.odom_subscriber = rospy.Subscriber('/gazebo/model_states', Odometry, self.proportional_controller)

        self.x = 0
        self.y = 0
        self.theta = 0

        # Default parameters: CHANGE HERE
        self.x_goal = 5     
        self.y_goal = 5
        self.yaw_gain = 0.5
        self.forward_gain = 0.2
        self.angular_limit = 2
        self.distance_threshold = 0.1


        self.rate = rospy.Rate(10)

    def update_pose_info(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        x = data.pose[1].position.x
        y = data.pose[1].position.y

        roll, pitch, yaw = quaternion_to_euler(data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w)
        self.x = x
        self.y = y
        self.theta = yaw
        
        # go to goal position
        # self.move2goal()

        """Moves the turtle to the goal."""
        # Get the input from the user.
        # Please, insert a number slightly greater than 0 (e.g. 0.01).

        goal_x = self.x_goal
        goal_y = self.y_goal


        vel_msg = Twist()

        # kro=0.2
        # kalpha=0.5
        # kbeta=-0.05

        # control parameters
        wgain = self.yaw_gain  # Angular velocity gain
        vconst = self.forward_gain # Linear velocity gain
        angular_limit = self.angular_limit    # Angular velocity limits


        # robot state
        x_state = x
        y_state = y
        theta = yaw

        # start: proportional control for robot

        v = 0   # Linear velocity
        w = 0   # Angle velocity
        distance = sqrt((goal_y - y_state)**2+(goal_x - x_state)**2)
        print "distance = ", distance

        if distance >= self.distance_threshold:

            v = vconst
            desireYaw = atan2(goal_y-y, goal_x-x)
            u = desireYaw-theta

            # bound for differential wheel contol
            bound = atan2(sin(u),cos(u))
            w = min(angular_limit , max(-1*angular_limit, wgain*bound))

            # print('xd,yd: (%f,%f), xr,yr: (%f, %f), distance: %f, angle: %f' % (goal_x, goal_y, self.x, self.y, distance, u))

            # Linear velocity in the x-axis.
            vel_msg.linear.x = distance*vconst
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = w

            # print(vel_msg)
            # print(self.x, self.y, self.theta)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            # self.rate.sleep()


        else:
            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

        # self.rate.sleep()

def quaternion_to_euler(x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        X = math.atan2(t0, t1)
        Y = math.asin(t2)
        Z = math.atan2(t3, t4)

        return X, Y, Z



if __name__ == '__main__':
    try:
        x = MyRobot() 
        rospy.spin()      
    except rospy.ROSInterruptException:
        print "error!"