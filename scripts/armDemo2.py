#!/usr/bin/env python

import time
import rospy

from numpy import append

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

FLYING_LEFT = [0.09, -0.35, 1.93, 0.4, 0.0, 0.0, 0.0]
FLYING_RIGHT = [-0.09, 0.35, 1.93, -0.4, 0.0, 0.0, 0.0]

HOME_LEFT = [0.1, -1.3, 1.94, 1.18, 0.0, -0.07, 0.0]
HOME_RIGHT = [-0.1, 1.3, 1.94, -1.18, 0.0, 0.07, 0.0]

DOWN_LEFT = [-0.02, -1.07, 1.56, 0.53, 0.0, 0.0, 0.0]
DOWN_RIGHT = [0.02, 1.07, 1.56, -0.53, 0.0, 0.0, 0.0]

ZERO_VECTOR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

RIGHT_NAMES = None
LEFT_NAMES = None

ROBOT_NAME = None

def sendRightArmTrajectory():
    msg = JointTrajectory()
    msg.joint_names = RIGHT_NAMES

    trajectoryPoints = [createTrajectoryPoint(2.0, FLYING_RIGHT),
                        createTrajectoryPoint(4.0, DOWN_RIGHT),
                        createTrajectoryPoint(6.0, FLYING_RIGHT),
                        createTrajectoryPoint(8.0, DOWN_RIGHT),
                        createTrajectoryPoint(10.0, HOME_RIGHT)]
    msg.points = trajectoryPoints

    rospy.loginfo( 'publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

def createTrajectoryPoint(time, positions):
	point = JointTrajectoryPoint()
	point.time_from_start = rospy.Duration(time)
	point.positions = positions
	point.velocities = ZERO_VECTOR
	return point

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo2')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run armDemo2.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

            right_arm_joint_parameter_names = "/ihmc_ros/{0}/right_arm_joint_names".format(ROBOT_NAME)
            left_arm_joint_parameter_names = "/ihmc_ros/{0}/left_arm_joint_names".format(ROBOT_NAME)

            if rospy.has_param(right_arm_joint_parameter_names) and rospy.has_param(left_arm_joint_parameter_names):
                RIGHT_NAMES = rospy.get_param(right_arm_joint_parameter_names)
                LEFT_NAMES = rospy.get_param(left_arm_joint_parameter_names)

                armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_joint_trajectory2".format(ROBOT_NAME), JointTrajectory, queue_size=1)

                rate = rospy.Rate(10) # 10hz
                time.sleep(1)

                # make sure the simulation is running otherwise wait
                if armTrajectoryPublisher.get_num_connections() == 0:
                    rospy.loginfo( 'waiting for subsciber...')
                    while armTrajectoryPublisher.get_num_connections() == 0:
                        rate.sleep()

                if not rospy.is_shutdown():
                    sendRightArmTrajectory()
                    time.sleep(2)
            else:
                if not rospy.has_param(left_arm_joint_parameter_names):
                    rospy.logerr("Missing parameter {0}".format(left_arm_joint_parameter_names))
                if not rospy.has_param(right_arm_joint_parameter_names):
                    rospy.logerr("Missing parameter{0}".format(right_arm_joint_parameter_names))

    except rospy.ROSInterruptException:
        pass
