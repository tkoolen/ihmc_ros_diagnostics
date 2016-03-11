#!/usr/bin/env python

import time
import rospy

from numpy import append

from ihmc_msgs.msg import ArmJointTrajectoryPacketMessage
from ihmc_msgs.msg import JointTrajectoryPointMessage

LEFT_HOME = [0.1, -1.3, 1.94, 1.18, 0.0, -0.07, 0.0]
RIGHT_HOME = [-0.1, 1.3, 1.94, -1.18, 0.0, 0.07, 0.0]
ZERO_VECTOR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ELBOW_BENT_UP = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]

ROBOT_NAME = None

def sendRightArmTrajectory():
    msg = ArmJointTrajectoryPacketMessage()

    trajectoryPoints = [createTrajectoryPoint(2.0, ZERO_VECTOR),
    					createTrajectoryPoint(3.0, ELBOW_BENT_UP),
    					createTrajectoryPoint(4.0, ZERO_VECTOR),
    					createTrajectoryPoint(6.0, RIGHT_HOME)]

    msg.robot_side = ArmJointTrajectoryPacketMessage.RIGHT
    msg.trajectory_points = trajectoryPoints

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

def sendLeftArmTrajectory():
    msg = ArmJointTrajectoryPacketMessage()

    trajectoryPoints = [createTrajectoryPoint(2.0, ZERO_VECTOR),
    					createTrajectoryPoint(4.0, LEFT_HOME)]

    msg.robot_side = ArmJointTrajectoryPacketMessage.LEFT
    msg.trajectory_points = trajectoryPoints

    rospy.loginfo('publishing left trajectory')
    armTrajectoryPublisher.publish(msg)

def createTrajectoryPoint(time, positions):
	point = JointTrajectoryPointMessage()
	point.time = time
	point.positions = positions
	point.velocities = ZERO_VECTOR
	return point

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_joint_trajectory".format(ROBOT_NAME), ArmJointTrajectoryPacketMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subsciber...')
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
            time.sleep(2)

        if not rospy.is_shutdown():
            sendLeftArmTrajectory()
            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
