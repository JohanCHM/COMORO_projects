#!/usr/bin/env python

# ====================================================================
#  HW02-Task03: Script to track the IMU data sensor 
# ====================================================================
# Scrippt to:
#  * Keep track of the sensor data from a gps
#  * Capture the data from the GPS sensor once the robot have reach an specific position into a csv file
#  * Controll the robot to move across an environment following the waypoints given
# 
#   This program was also used in "HW02-Task04: Watch the camera feed" and
#   "HW02-Task05: Visualize the scanner data"
#
# author: Carlos Hansen Mendoza <carlos.hansen@post.au.dk>
# date: october-2019

import rospy
import tf               # for doing angular transformations#
import sensor_msgs.msg  # for the manipulation of sensor data
import nav_msgs.msg     # for the namipulation of
import geometry_msgs.msg    # for the manipulation of messages to comand the robot

import math
import csv          # for saving data into a csv file


class MonitorGpsRobot:
    def __init__(self):
        # ---Tunable parameters---

        # Waypoints to visit
        self.waypoints = [[0, 0],
                          [1, 0],
                          [11, 0],
                          [11, 5],
                          [4, 5]]

        self.inReach = 0.2  # Error distance to consider that the robot has reach the point

        # Control constants for the controller
        self.kr = 0.18  # kr > 0
        self.ka = 0.2  # (ka-kr) > 0	# ka must be greater than kr
        self.kb = -0.05  # kb < 0

        self.angular_limit = 2

        # name of the file to store the gps information. Latitude, Longitude, Altitude
        self.fileName = 'gps_task03.csv'

        # ---Variables to internal calculation ---
        self.currentGoal = self.waypoints[0]   # Waypoint to reach
        self.waypointIndex = 0      # Index to track current waypoints

        self.currentLatitude = 0.0
        self.currentLongitude = 0.0
        self.currentAltitude = 0.0
        self.gpsData = [[self.currentLatitude,
                        self.currentLongitude, self.currentAltitude]]  # Initial values of the GPS data

        #Initialize the CSV
        with open(self.fileName, mode='w') as data_file:
                    file_writter = csv.writer(
                        data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    file_writter.writerow(self.gpsData[0])

        # Initialize the node
        rospy.init_node('robot_controller', anonymous=True)

        #------- To track the GPS sensor
        # Every time a navsat/fix is read it update the current position values
        self.gpsSubscriber = rospy.Subscriber(
            'navsat/fix', sensor_msgs.msg.NavSatFix, self.savingGPS)

        #------- To control the position
        self.cmdMsg = geometry_msgs.msg.Twist()

        # Publisher to command the robot via the topic /cmd_vel
        self.cmdPub = rospy.Publisher(
            '/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # every time the ofometry filtered is received the control parameters get updated
        self.poseForControlSubscriber = rospy.Subscriber(
            'odometry/filtered', nav_msgs.msg.Odometry, self.robotController)

        # Subscriber to check if the corner had been reach and change the goal corner to go
        # When the corner is reached the GPS gets print and saved
        self.poseForPathSubscriber = rospy.Subscriber(
            'odometry/filtered', nav_msgs.msg.Odometry, self.controlGoal)

        self.rate = rospy.Rate(10)

    def savingGPS(self, message):
        """"Function to save the Linear Acceleration and Angular Velocity"""

        # Saving GPS data
        self.currentLatitude = message.latitude
        self.currentLongitude = message.longitude
        self.currentAltitude = message.altitude

        self.rate.sleep()

    def robotController(self, message):
        """"Function to control the robot"""
        # Get the pose from the message
        pos = message.pose.pose
        quat = pos.orientation

        # Transform quaternion coordinates to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                           quat.z, quat.w))

        theta = angles[2]
        # X, Y, Theta
        robotPos = [pos.position.x, pos.position.y, theta]

        # Calculate RAB based on the previous robot position and the desired position
        rho = math.sqrt((robotPos[0]-self.currentGoal[0]) **
                        2+(robotPos[1]-self.currentGoal[1])**2)
        alpha = -theta + \
            math.atan2(self.currentGoal[1]-robotPos[1],
                       self.currentGoal[0]-robotPos[0])
        beta = - theta - alpha

        # Velocities
        v = self.kr * rho
        u = alpha

        w = math.atan2(math.sin(u), math.cos(u))
        # w = min(self.angular_limit , max(-1*self.angular_limit, self.ka * bound))

        # Publish
        self.cmdMsg.linear.x = v
        self.cmdMsg.angular.z = w
        self.cmdPub.publish(self.cmdMsg)

        # print("Goal {0} distance: {1}  speed: {2} w:{3}".format(
        #     self.currentGoal, rho, v, w))

    def controlGoal(self, message):
        """Function to measure the current distannce to the goal and if reached, update the next goal"""
        # Get the pose from the message
        pos = message.pose.pose
        robotPos = [pos.position.x, pos.position.y]

        distance = math.sqrt((robotPos[0]-self.currentGoal[0]) ** 2
                             + (robotPos[1]-self.currentGoal[1]) ** 2)

        if (distance < self.inReach):
            # Save and Print GPS Position
            print("Point: {point} Latitude: {lat} Longitude: {lon} Altitude: {alt}".format(
                point=self.currentGoal, lat=self.currentLatitude, lon=self.currentLongitude, alt=self.currentAltitude))

            robotGPS = [self.currentLatitude,
                        self.currentLongitude, self.currentAltitude]

            with open(self.fileName, mode='a') as data_file:
                file_writter = csv.writer(
                    data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                file_writter.writerow(robotGPS)

            # Update the waypoint 
            self.waypointIndex = self.waypointIndex + 1
            if (self.waypointIndex >= len(self.waypoints)):
                self.waypointIndex = 0
            self.currentGoal = self.waypoints[self.waypointIndex]

########################################
if __name__ == "__main__":
    try:
        task = MonitorGpsRobot()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("error!")
