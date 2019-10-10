#!/usr/bin/env python

import rospy
import tf               # for doing angular transformations#
import sensor_msgs.msg  # for the manipulation of sensor data
import nav_msgs.msg     # for the namipulation of
import geometry_msgs.msg    # for the manipulation of messages to comand the robot

import math
import csv          # for saving data into a csv file


class MonitorImuRobot:
    def __init__(self):
        # ---Tunable parameters---

        # Waypoints to visit
        self.waypoints = [[0, 0],
                          [1, 0],
                          [11,0],
                          [11,5],
                          [4,5]]
        
        self.inReach = 0.2  # Error distance to consider that the robot has reach the point

        # Control constants for the controller
        self.kr = 0.18  # kr > 0
        self.ka = 0.2  # (ka-kr) > 0	# ka must be greater than kr
        self.kb = -0.05  # kb < 0

        self.angular_limit = 2

        # name of the file to store the imu data vector3 lin_Acceleration, vector3: Angular_velocity
        self.fileName = 'imu_task02.csv'

        # ---Variables to internal calculation ---
        self.currentGoal = self.waypoints[0]   # Waypoint to reach
        self.waypointIndex = 0      # Index to track current waypoints

        self.imuData = [[0, 0, 0, 0, 0, 0]]  # Initial values of the IMU data



        #Initialize the CSV
        with open(self.fileName, mode='w') as data_file:
                    file_writter = csv.writer(
                        data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    file_writter.writerow(self.imuData[0])

        # Initialize the node
        rospy.init_node('robot_controller', anonymous=True)

        #------- To track the IMU sensor
        # Every time an imu/data is read its info gets check to see if it must be saved
        self.imuSubscriber = rospy.Subscriber(
            'imu/data', sensor_msgs.msg.Imu, self.savingIMU)

        #------- To control the position
        self.cmdMsg = geometry_msgs.msg.Twist()

        # Publisher to command the robot via the topic /cmd_vel
        self.cmdPub = rospy.Publisher(
            '/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # every time the ofometry filtered is received the control parameters get updated
        self.poseForControlSubscriber = rospy.Subscriber(
            'odometry/filtered', nav_msgs.msg.Odometry, self.robotController)

        # Subscriber to check if the corner had been reach and change the goal corner to go
        self.poseForPathSubscriber = rospy.Subscriber(
            'odometry/filtered', nav_msgs.msg.Odometry, self.controlGoal)

    def savingIMU(self, message):
        """"Function to save the Linear Acceleration and Angular Velocity"""

        # Spliting IMU data
        lin_acel = message.linear_acceleration
        ang_vel = message.angular_velocity

        robotImu = [lin_acel.x, lin_acel.y, lin_acel.z,
                    ang_vel.x, ang_vel.y, ang_vel.z]

        #If the change in pose is significant
        if (checkAtLeast2Changes(0.02, self.imuData[-1], robotImu)):

            self.imuData.append(robotImu)

            # print(len(self.imuData))
            # print(robotImu)

            with open(self.fileName, mode='a') as data_file:
                file_writter = csv.writer(
                    data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                file_writter.writerow(robotImu)

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

        w = math.atan2(math.sin(u),math.cos(u))
        # w = min(self.angular_limit , max(-1*self.angular_limit, self.ka * bound))

        # Publish
        self.cmdMsg.linear.x = v
        self.cmdMsg.angular.z = w
        self.cmdPub.publish(self.cmdMsg)

        print ("Goal {0} distance: {1}  speed: {2} w:{3}".format(self.currentGoal,rho,v,w))


    def controlGoal(self, message):
        """Function to measure the current distannce to the goal and if reached, update the next goal"""
        # Get the pose from the message
        pos = message.pose.pose
        robotPos = [pos.position.x, pos.position.y]

        distance = math.sqrt((robotPos[0]-self.currentGoal[0]) ** 2
                             + (robotPos[1]-self.currentGoal[1])** 2)

        if (distance < self.inReach):
            self.waypointIndex = self.waypointIndex + 1 
            if (self.waypointIndex >= len(self.waypoints)):
                self.waypointIndex = 0
            self.currentGoal = self.waypoints[self.waypointIndex]



def checkAtLeast2Changes(amount, oldData, newData):
    """Check if at least to values from an old and a new IMU simplified vector have change a certain amount"""

    cnt = 0  # counter to know how many changes where
    if(abs(oldData[0]-newData[0]) > amount):
        cnt = cnt + 1
    if(abs(oldData[1]-newData[1]) > amount):
        cnt = cnt + 1
    if(abs(oldData[2]-newData[2]) > amount):
        cnt = cnt + 1
    if(abs(oldData[3]-newData[3]) > amount):
        cnt = cnt + 1
    if(abs(oldData[4]-newData[4]) > amount):
        cnt = cnt + 1
    if(abs(oldData[5]-newData[5]) > amount):
        cnt = cnt + 1

    if (cnt >= 2):
        return True
    else:
        return False


########################################
if __name__ == "__main__":
    try:
        task = MonitorImuRobot()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("error!")
