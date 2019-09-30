#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
import nav_msgs.msg
import math
import matplotlib.pyplot as plt


def robotCallback(message, cbk_args):
    # Callback arguments
    pub, msg, goal, time, velocity, planned, real = cbk_args

    now = rospy.get_time()
    # Current simulation time
    if now - time[0] > 1:
        time[0] = now
        time[1] = time[1] + 1

        # # Linear
        # goal[0] = time[1]
        # goal[1] = time[1]

        # Circular
        # xCenter = 5 
        # yCenter = 0
        # radius = 5

        # spin = time[1] % (4*radius)

        # if spin <= (xCenter + radius) :
        #     goal[0] = spin
        #     goal[1] = math.sqrt(abs((goal[0]-xCenter) ** 2 - radius**2)) + yCenter
        # else:
        #     goal[0] = 2 * (xCenter + radius) - spin
        #     goal[1] = -math.sqrt(abs((goal[0]-xCenter) ** 2 - radius**2)) + yCenter

        # 8 loop
        xCenter = 5
        yCenter = 0
        radius = 5

        spin = time[1] % (8*radius)

        if spin <= (xCenter + radius) :
            goal[0] = spin
            goal[1] = math.sqrt(abs((goal[0]-xCenter) ** 2 - radius**2)) + yCenter
        elif spin <= 2*(xCenter + radius):
            goal[0] = spin
            goal[1] = -math.sqrt(abs((goal[0]-(xCenter+2*radius)) ** 2 - radius**2)) + yCenter
        elif spin <= 3*(xCenter + radius):
            goal[0] = 8*radius-spin
            goal[1] = math.sqrt(abs((goal[0]-(xCenter+2*radius)) ** 2 - radius**2)) + yCenter
        else:
            goal[0] = 8*radius-spin
            goal[1] = math.sqrt(abs((goal[0]-(xCenter+2*radius)) ** 2 - radius**2)) + yCenter
               

            

    vConst = velocity  # Linear velocity

    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation

    # From quaternion to Euler
    # TODO: Change for Ilkar equation
    angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                       quat.z, quat.w))

    theta = angles[2]
    robotPos = [pos.position.x, pos.position.y, theta]  # X, Y, Theta

    # Calculate RAB based on the previous robot position and the desired position
    rho = math.sqrt((robotPos[0]-goal[0])**2+(robotPos[1]-goal[1])**2)
    alpha = -theta + math.atan2(goal[1]-robotPos[1], goal[0]-robotPos[0])
    beta = - theta - alpha

    print(alpha)

    # Next robot position
    # x_nxt = robotPos[0] + kr * rho * math.cos(-robotPos[2])
    # y_nxt = robotPos[1] + kr * rho * math.sin(-robotPos[2])
    # theta_nxt = robotPos[2] + (ka * alpha + kb*beta)

    # Publish
    msg.linear.x = vConst
    msg.angular.z = alpha
    pub.publish(msg)

    planned = plt.plot(goal[0], goal[1],'g-')
    real = plt.plot(robotPos[0], robotPos[1], 'r-.')
    plt.draw() 

    # Reporting
    # print(goal)
    print('Location: x=%4.1f,y=%4.1f, goalX=%4.1f, goalY=%4.1f rho=%4.2f, alpha=%4.2f, beta=%4.2f' %
          (robotPos[0], robotPos[1], goal[0], goal[1], rho, alpha, beta))

    # rate.sleep()


#####
# Initialize the node
rospy.init_node('robot_controller', anonymous=True)

rate = rospy.Rate(10)  # 10 Hz

time = [0, 0]  # [real_time, sim_time]


# Tunable parameters
kr = 0.1  # kr > 0
ka = 0.2  # (ka-kr) > 0	# ka must be greater than kr
kb = -0.05  # kb < 0

# Linear velocity for the robot
robotVelocity = 7.5

# Plot
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(15, 15)
ax_anim = plt.axes(xlim=(-10, 20), ylim=(-20,20))	#semisinusoid

plt.title("Robot movement trying to follow the path") 


# Initial rho, alpha, beta
goal = [0, 0]  # Goal

planned = plt.plot(goal[0], goal[1],'g-')
real = plt.plot(0, 0, 'r-.')

# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as
# additional parameters to the callback function.

rospy.Subscriber('odometry/filtered', nav_msgs.msg.Odometry, robotCallback,
                 (cmdpub, cmdmsg, goal, time, robotVelocity,planned,real))


rospy.spin()
