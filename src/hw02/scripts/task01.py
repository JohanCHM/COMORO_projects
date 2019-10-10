#!/usr/bin/env python

import rospy
import tf               # for doing angular transformations
import nav_msgs.msg     # for the manipulation of navigation data

import csv          # for saving data into a csv file


class MonitorOdometryRobot:

    def __init__(self):
        self.poses = [[0, 0, 0]]  # variable to store poses

        self.fileName = 'poses_task01.csv'

        # Initialize the node
        rospy.init_node('robot_controller', anonymous=True)

        # Every time an /odometry/message is received its info gets check to see if it must be saved
        self.poseSubscriber = rospy.Subscriber(
            'odometry/filtered', nav_msgs.msg.Odometry, self.savingInfo)

    def savingInfo(self, message):
        """"Function to save into a veriable and print"""

        # Generate a simplified pose
        pos = message.pose.pose
        quat = pos.orientation

        # From quaternion to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                           quat.z, quat.w))

        theta = angles[2]   #just the yaw

        robotPos = [pos.position.x, pos.position.y, theta]  # X, Y, Theta

        #If the change in pose is significant
        if ((abs(self.poses[-1][0] - robotPos[0]) > 0.1) or
            (abs(self.poses[-1][1] - robotPos[1]) > 0.1) or
                (abs(self.poses[-1][2] - robotPos[2]) > 0.05)):

            self.poses.append(robotPos)

            print(robotPos)

            with open(self.fileName, mode='a') as data_file:
                file_writter = csv.writer(
                    data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                file_writter.writerow(robotPos)


########################################


if __name__ == "__main__":
    try:
        task = MonitorOdometryRobot()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("error!")
