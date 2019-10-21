#!/usr/bin/env python

import rospy
import tf
import nav_msgs.msg
from std_msgs.msg import Empty
import geometry_msgs.msg    # for the manipulation of messages to comand the robot
import sensor_msgs.msg

import math

class OdometryDrone:
    def __init__(self):

        # ---- Parameters to modify
        self.waypoints = [[0,0,2],
                    [0,6,2],
                    [6,6,2],
                    [6,0,2]]

        
        # Initialize the node
        rospy.init_node('drone_controller', anonymous=True)

        #------- To control the position
        self.cmdMsg = geometry_msgs.msg.Twist()

        # Publisher to command the robot via the topic /cmd_vel
        self.cmdPub = rospy.Publisher(
            '/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # every time the odometry filtered is received the control parameters get updated
        self.poseForControlSubscriber = rospy.Subscriber(
            'bebop/land', nav_msgs.msg.Odometry, self.droneController)

        #-------  Drone actions ------ 
        # Takeoff of the drone
        self.takeoff(10)
        # Land
        # self.land(10)

    @staticmethod
    def takeoff(takeoffTime):
        """ Publication of the message to takeoff during the specified time"""
        print ("Takingoff")
        rate = rospy.Rate(10) # 10hz

        takeoffPub = rospy.Publisher("bebop/takeoff", Empty, queue_size=1 )
        
        beginTime = rospy.Time.now() # Starting time
        secondsTakeoff = rospy.Duration(takeoffTime) # Desired duration in terms of ros
        endTime = secondsTakeoff + beginTime    # Desired duration in reference of begininig
        while rospy.Time.now() < endTime:   #publication of message while duration
            takeoffPub.publish(Empty())
            rate.sleep()

    @staticmethod
    def land(landTime):
        """ Publication of the message to land during the specified time"""
        print ("Landing")
        rate = rospy.Rate(10) # 10hz

        landPub = rospy.Publisher("bebop/land", Empty, queue_size=1 )
        
        beginTime = rospy.Time.now() # Starting time
        secondsTakeoff = rospy.Duration(landTime) # Desired duration in terms of ros
        endTime = secondsTakeoff + beginTime    # Desired duration in reference of begininig
        while rospy.Time.now() < endTime:   #publication of message while duration
            landPub.publish(Empty())
            rate.sleep()

    def waypoints2latLong(self):
        rospy.wait_for_message("",sensor_msgs.msg.NavSatFix)

    def droneController(self, message):
        """"Function to control the drone"""
        # Get the pose from the message
        pos = message.pose.pose
        quat = pos.orientation

        # Transform quaternion coordinates to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                           quat.z, quat.w))

        theta = angles[2]
        # X, Y, Theta
        dronePos = [pos.position.x, pos.position.y, theta]

        print(dronePos)

        # # Calculate RAB based on the previous robot position and the desired position
        # rho = math.sqrt((robotPos[0]-self.currentGoal[0]) **
        #                 2+(robotPos[1]-self.currentGoal[1])**2)
        # alpha = -theta + \
        #     math.atan2(self.currentGoal[1]-robotPos[1],
        #                self.currentGoal[0]-robotPos[0])
        # beta = - theta - alpha


        # # Velocities
        # v = self.kr * rho
        # u = alpha

        # w = math.atan2(math.sin(u),math.cos(u))
        # w = min(self.angular_limit , max(-1*self.angular_limit, self.ka * bound))

        # # Publish
        # self.cmdMsg.linear.x = v
        # self.cmdMsg.angular.z = w
        # self.cmdPub.publish(self.cmdMsg)

        # print ("Goal {0} distance: {1}  speed: {2} w:{3}".format(self.currentGoal,rho,v,w))
        

        

########################################


if __name__ == "__main__":
    try:
        task01 = OdometryDrone()
        # rospy.spin()

    except rospy.ROSInterruptException:
        print("error!")
