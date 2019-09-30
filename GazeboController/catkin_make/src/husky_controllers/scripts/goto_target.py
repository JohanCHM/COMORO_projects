#!/usr/bin/env python
# ====================================================================
# Feedback controler for a Husky robot to reach an specific point
# ====================================================================
#
# author: Carlos Hansen Mendoza
# date: september-2019
import rospy
import tf
import geometry_msgs.msg
import nav_msgs.msg
import math
import matplotlib.pyplot as plt

def move():
    # Initialize node
    rospy.init_node('goto_target', anonymous=True)

    # Tunable parameters--- Ask for control constants
    # kr = 0.18	# kr > 0
    kr = float(input("Insert the value of kr? (It has to be greater than 0): "))
    # ka = 0.2	# (ka-kr) > 0	# ka must be greater than kr
    ka = float(input("Insert the value of ka? (It has to be greater than kr): "))
    # kb = -0.05	# kb < 0
    kb = float(input("Insert the value of kb? (It has to be less than 0): "))

    # Target point
    target = [0, 0]
    target[0] = float(input("Insert the x coordinate of the target point?: "))
    target[1] = float(input("Insert the y coordinate of the target point?: "))


    robotCallback.fig = plt.figure()
    
    

    # Setup publisher
    cmdmsg = geometry_msgs.msg.Twist()
    cmdpub = rospy.Publisher(
        '/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    # Setup subscription
    rospy.Subscriber('odometry/filtered', nav_msgs.msg.Odometry, robotCallback,
                     (cmdpub, cmdmsg, target, kr, ka, kb))

    plt.show()

    # rospy.spin()


def robotCallback(message, cbk_args):
    # Callback arguments
    pub, msg, goal, kr, ka, kb = cbk_args

    rate = rospy.Rate(10)   #10 H

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
    rho = math.sqrt((robotPos[0]-goal[0])**2+(robotPos[1]-goal[1])**2)
    alpha = -theta + math.atan2(goal[1]-robotPos[1], goal[0]-robotPos[0])
    beta = - theta - alpha

    # print(alpha)

    # Next robot position
    # x_nxt = robotPos[0] + kr * rho * math.cos(-robotPos[2])
    # y_nxt = robotPos[1] + kr * rho * math.sin(-robotPos[2])
    # theta_nxt = robotPos[2] + (ka * alpha + kb*beta)

    # Velocities
    v = kr * rho
    w = alpha

    # Publish
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)

    # Output Status
    # print(goal)
    print('Location: x=%4.1f,y=%4.1f, goalX=%4.1f, goalY=%4.1f rho=%4.2f, alpha=%4.2f, beta=%4.2f' %
          (robotPos[0], robotPos[1], goal[0], goal[1], rho, alpha, beta))


    # Plot
    plt.title("Error vs time")
    plt.xlabel("time") 
    plt.ylabel("error") 
    plt.plot(rospy.get_time(), rho, '.')
    plt.draw()
    

    plt.draw() 

    rate.sleep()


if __name__ == "__main__":
    try:
        move()

    except rospy.ROSInterruptException:
        pass
