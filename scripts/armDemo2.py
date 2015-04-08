#!/usr/bin/env python

import time
import rospy

from numpy import append

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

LEFT = 0
RIGHT = 1

FLYING_LEFT = [0.09, -0.35, 1.93, 0.4, 0.0, 0.0, 0.0]
FLYING_RIGHT = [-0.09, 0.35, 1.93, -0.4, 0.0, 0.0, 0.0]

HOME_LEFT = [0.1, -1.3, 1.94, 1.18, 0.0, -0.07, 0.0]
HOME_RIGHT = [-0.1, 1.3, 1.94, -1.18, 0.0, 0.07, 0.0]

ZERO_VECTOR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

RIGHT_NAMES = ['r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2']
LEFT_NAMES = ['l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2']

def sendRightArmTrajectory():
    msg = JointTrajectory()
    msg.joint_names = RIGHT_NAMES

    trajectoryPoints = [createTrajectoryPoint(2.0, FLYING_RIGHT),
                        createTrajectoryPoint(4.0, DOWN_RIGHT),
                        createTrajectoryPoint(6.0, FLYING_RIGHT),
                        createTrajectoryPoint(8.0, DOWN_RIGHT),
                        createTrajectoryPoint(10.0, HOME_RIGHT)]
    msg.points = trajectoryPoints

    print 'publishing right trajectory'
    armTrajectoryPublisher.publish(msg)

def createTrajectoryPoint(time, positions):
	point = JointTrajectoryPoint()
	point.time_from_start = rospy.Duration(time)
	point.positions = positions
	point.velocities = ZERO_VECTOR
	return point

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_box_step')

        armTrajectoryPublisher = rospy.Publisher('/ihmc_ros/atlas/control/arm_joint_trajectory2', JointTrajectory, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            print 'waiting for subsciber...'
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass