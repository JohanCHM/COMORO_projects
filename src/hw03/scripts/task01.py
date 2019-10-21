#!/usr/bin/env python

import rospy
import tf
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg 
import geometry_msgs.msg    # for the manipulation of messages to comand the robot

import math
import collections

import csv          # for saving data into a csv file


class OdometryDrone:
    def __init__(self):

        #----- Modifiable Parameters
        # Control constants for the controller
        self.kxy = 0.05   # Proportional parameter for xy
        self.kz = 0.2   # Proportional parameter for z

        # ---- Parameters to modify
        self.waypoints = [[0,0,2],
                    [0,6,2],
                    [6,6,2],
                    [6,0,2]]

        self.currentGoal = self.waypoints[3]

        self.freq = 10  # Frequency of topic messages
        
        self.stableTime = 5 # Time in the vecinity of target waypoint
        self.inReach = 0.2    #Distance to be considered in the waypoint

        self.fileName = "task01.csv"

        # ----- Initialization stage

        self.currentGoal = self.waypoints[0]   # Waypoint to reach
        self.waypointIndex = 0      # Index to track current waypoints

        # List to have a mean of messages to make sure the distance do not oscillate much
        self.thresholdList = collections.deque(maxlen=(self.stableTime*self.freq))

        #Initialize the CSV
        with open(self.fileName, mode='w') as data_file:
                    file_writter = csv.writer(
                        data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    file_writter.writerow([0,0,0,0,0])

        # Initialize the node
        rospy.init_node('drone_controller', anonymous=True)
        self.rate = rospy.Rate(self.freq)  # 10hz

        # To control the position
        self.cmdMsg = geometry_msgs.msg.Twist()

        # Publisher to command the robot via the topic /cmd_vel
        self.cmdPub = rospy.Publisher(
            '/bebop/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # ------ Getting the initial parameters of the drone
        initialOdom = rospy.wait_for_message('bebop/odom', nav_msgs.msg.Odometry,)
        initialInertialCoord = self.odometryMsg2InertialCoordinates(initialOdom)
 
        self.bebopAngled = initialInertialCoord[3]# angle between body and bebop orientation


        # ------ Visualization of trajectories for RVIZ
        self.rvizMsg = nav_msgs.msg.Odometry()
        self.rvizHeader = std_msgs.msg.Header()
        self.rvizHeader.frame_id = "/odom"

        # Publisher for visualization
        self.rvizPub = rospy.Publisher(
            'drone_odometry', nav_msgs.msg.Odometry, queue_size=10)

        # Subscriber to check for every pose into a messge for rviz
        self.rvizSubscriber = rospy.Subscriber(
            'bebop/odom', nav_msgs.msg.Odometry, self.robotPosOr)


        # ------ Update target
        # Subscriber to check if the corner had been reach and change the goal corner to go
        self.poseForPathSubscriber = rospy.Subscriber(
            'bebop/odom', nav_msgs.msg.Odometry, self.controlGoal)

        # ------ Drone commands

        # Takeoff of the drone
        self.takeoff(5)

        # every time the odometry filtered is received the control parameters get updated
        while not rospy.is_shutdown():
            # print("----")
            self.poseForControlSubscriber = rospy.Subscriber(
                'bebop/odom', nav_msgs.msg.Odometry, self.droneController)
            self.rate.sleep()
            
                

        # Land
        # self.land(10)

    def takeoff(self, takeoffTime):
        """ Publication of the message to takeoff during the specified time"""
        print("Takingoff")
    

        takeoffPub = rospy.Publisher("bebop/takeoff", std_msgs.msg.Empty, queue_size=1)

        beginTime = rospy.Time.now()  # Starting time
        # Desired duration in terms of ros
        secondsTakeoff = rospy.Duration(takeoffTime)
        # Desired duration in reference of begininig
        endTime = secondsTakeoff + beginTime
        while rospy.Time.now() < endTime:  # publication of message while duration
            takeoffPub.publish(std_msgs.msg.Empty())
            self.rate.sleep()

    
    def land(self, landTime):
        """ Publication of the message to land during the specified time"""
        print("Landing")
        
        landPub = rospy.Publisher("bebop/land", std_msgs.msg.Empty, queue_size=1)

        beginTime = rospy.Time.now()  # Starting time
        # Desired duration in terms of ros
        secondsTakeoff = rospy.Duration(landTime)
        # Desired duration in reference of begininig
        endTime = secondsTakeoff + beginTime
        while rospy.Time.now() < endTime:  # publication of message while duration
            landPub.publish(std_msgs.msg.Empty())
            self.rate.sleep()

    def droneController(self, message):
        """"Control the drone"""

        dronePosInertial = self.odometryMsg2InertialCoordinates(message)
        
        # Calculate the error
        rhoX = self.currentGoal[0] - dronePosInertial[0]
        rhoY = self.currentGoal[1] - dronePosInertial[1]
        rhoZ = self.currentGoal[2] - dronePosInertial[2]

        # Create command
        self.cmdMsg.linear.x = self.kxy * rhoX
        self.cmdMsg.linear.y = self.kxy * rhoY
        self.cmdMsg.linear.z = self.kz * rhoZ

        # Publish
        self.cmdPub.publish(self.cmdMsg)
        


    def controlGoal(self, message):
        """Measure the current distance to the goal and if reached, update the next goal. Then save it to an CSV file"""
        
        dronePosInertial = self.odometryMsg2InertialCoordinates(message)

        distance = math.sqrt((dronePosInertial[0]-self.currentGoal[0]) ** 2
                             + (dronePosInertial[1]-self.currentGoal[1])** 2
                             + (dronePosInertial[2]-self.currentGoal[2])** 2)

        self.thresholdList.append(distance) # Save into the list
        

        if self.thresholdList > 0:
            average = sum(self.thresholdList) / len(self.thresholdList)

        if (average < self.inReach):
            self.waypointIndex = self.waypointIndex + 1 
            if (self.waypointIndex >= len(self.waypoints)):
                self.waypointIndex = 0
            self.currentGoal = self.waypoints[self.waypointIndex]

        print("X:{x} Y:{y} Z: {r} PSI:{p}".format(
            x=dronePosInertial[0], y=dronePosInertial[1], r=dronePosInertial[2], p=dronePosInertial[3]))
        print("Goal: {g} Average Distance: {a} Distances averaged: {l}".format(g=self.currentGoal, a=average, l=len(self.thresholdList)))


        # Saving to file

        dronePosInertial.append(distance)
        dataToSave = dronePosInertial

        # print(dataToSave)

        with open(self.fileName, mode='a') as data_file:
            file_writter = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            file_writter.writerow(dataToSave)


    def robotPosOr(self, message):
        """Visualization of the odometry nav msgs/Odometry"""  
        dronePosInertial = self.odometryMsg2InertialCoordinates(message)  

        # constructing poses part of the message
        self.rvizMsg.pose.pose.position.x = dronePosInertial[0]
        self.rvizMsg.pose.pose.position.y = dronePosInertial[1]
        self.rvizMsg.pose.pose.position.z = dronePosInertial[2]

        # Orientations into quaternions
        quat = tf.transformations.quaternion_from_euler(
            0, 0, dronePosInertial[3] - self.bebopAngled, 'ryxz')

        # constructing orientation part of the message
        self.rvizMsg.pose.pose.orientation.x = quat[0]
        self.rvizMsg.pose.pose.orientation.y = quat[1]
        self.rvizMsg.pose.pose.orientation.z = quat[2]
        self.rvizMsg.pose.pose.orientation.w = quat[3]

        # Update of the Header
        self.rvizHeader.stamp = rospy.Time.now()
        self.rvizMsg.header = self.rvizHeader

        # Publish 
        self.rvizPub.publish(self.rvizMsg)




    @staticmethod
    def odometryMsg2InertialCoordinates(message):
        """Convert an Odometry message to Inertial Coordinates (x,y,z,psi)"""
        # Get the pose from the message
        pos = message.pose.pose
        quat = pos.orientation
        
        # Transform quaternion coordinates to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                           quat.z, quat.w))

        psi = angles[2]
        # X, Y, psi
        dronePos = [pos.position.x, pos.position.y, pos.position.z, psi]
 
        # Rotation of reference Theta and Phi equal 0
        transX = math.cos(psi)*dronePos[0] + math.sin(psi)*dronePos[1]
        transY = -math.sin(psi)*dronePos[0] + math.cos(psi)*dronePos[1]
        transZ = dronePos[2]

        dronePosInertial = [transX, transY, transZ, psi]

        return dronePosInertial

    

        


########################################
if __name__ == "__main__":
    try:
        task01 = OdometryDrone()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("error!")
