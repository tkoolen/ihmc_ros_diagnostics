#!/usr/bin/env python

import time
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

def stepInPlace():
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5
    msg.execution_mode = 0
    msg.unique_id = -1
    step_width = 0.3

    rightFootFrame = Header()
    rightFootFrame.frame_id = RIGHT_FOOT_FRAME_NAME

    for x in xrange(3): # TODO
        msg.footstep_data_list.append(createFootstep(LEFT, PointStamped(rightFootFrame, Point(0., step_width, 0.))))
        msg.footstep_data_list.append(createFootstep(RIGHT, PointStamped(rightFootFrame, Point(0., 0., 0.))))

    footStepListPublisher.publish(msg)
    rospy.loginfo('walking in place...')
    waitForFootsteps(len(msg.footstep_data_list))

def boxStep():
    rospy.loginfo('start of boxStep.')
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5
    msg.execution_mode = 0
    msg.unique_id = -1

    rightFootFrame = Header()
    rightFootFrame.frame_id = RIGHT_FOOT_FRAME_NAME

    step_width_small = 0.3
    step_width_big = 0.4
    stride_length = 0.2

    for x in xrange(2):
        msg.footstep_data_list.append(createFootstep(LEFT, PointStamped(rightFootFrame, Point(stride_length, step_width_small, 0.))))
        msg.footstep_data_list.append(createFootstep(RIGHT, PointStamped(rightFootFrame, Point(stride_length, step_width_small - step_width_big, 0.))))
        msg.footstep_data_list.append(createFootstep(LEFT, PointStamped(rightFootFrame, Point(stride_length, 2 * step_width_small - step_width_big, 0.))))
        msg.footstep_data_list.append(createFootstep(RIGHT, PointStamped(rightFootFrame, Point(0., step_width_small - step_width_big, 0.))))
        msg.footstep_data_list.append(createFootstep(LEFT, PointStamped(rightFootFrame, Point(0., step_width_small, 0.))))
        msg.footstep_data_list.append(createFootstep(RIGHT, PointStamped(rightFootFrame, Point(0., 0., 0.))))

    footStepListPublisher.publish(msg)
    rospy.loginfo('box stepping...')
    waitForFootsteps(len(msg.footstep_data_list))

# Create a new footstep at the given position, with the same orientation as the stance foot
def createFootstep(stepSide, position):
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    # transform position to world
    positionWorld = tfBuffer.transform(position, 'world')
    footstep.location.x = positionWorld.point.x
    footstep.location.y = positionWorld.point.y
    footstep.location.z = positionWorld.point.z

    # take orientation from stance foot
    if stepSide == LEFT:
        stanceFootFrame = RIGHT_FOOT_FRAME_NAME
    else:
        stanceFootFrame = LEFT_FOOT_FRAME_NAME

    stanceFootToWorld = tfBuffer.lookup_transform('world', stanceFootFrame, rospy.Time())
    footstep.orientation = stanceFootToWorld.transform.rotation

    return footstep

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    rospy.loginfo('finished set of steps')

def receivedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_box_step')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run boxStep.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

            right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
            left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)

            if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, receivedFootStepStatus)
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)

                tfBuffer = tf2_ros.Buffer()
                tfListener = tf2_ros.TransformListener(tfBuffer)

                rate = rospy.Rate(10) # 10hz
                time.sleep(1)

                # make sure the simulation is running otherwise wait
                if footStepListPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subsciber...')
                    while footStepListPublisher.get_num_connections() == 0:
                        rate.sleep()

                if not rospy.is_shutdown():
                    stepInPlace()

                time.sleep(1)

                if not rospy.is_shutdown():
                    boxStep()

            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass

