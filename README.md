#ihmc\_ros\_diagnostics

Diagnostics tools for analyzing the IHMC ROS distribution

## Logging

The IHMC Controller is capable of generating custom logs that capture time-series for nearly every variable on the running robot as well as the IHMC control algorithm (tens of thousands of variables hundreds of times a second). We run this software every single time we turn the robot on; it is not required of you to do so, but it is useful in diagnosing bugs you may encounter.

###Usage

`roslaunch ihmc_ros_diagnostics ihmc_logger.launch`: Run the IHMC Logging software for logging data from the controller.

You can set the following roslaunch args:

- `ihmc_network_file:=<absolute path to network file>`: Specific the network configuration .ini file for the IHMC software. See [the GitHub pages documentation](TODO fill this in) for more information

## Box Step demo

The Box Step demo is a simple test/example of commanding the robot using Footstep Lists from `ihmc_msgs`. It can be used as a pattern matching example and also to vet the walking and balancing algorithm on hardware. To use this script properly, there are a few ROS paramters that need to be set:

- `/ihmc_ros/robot_name`
- `/ihmc_ros/<robot name>/left_foot_frame_name`
- `ihmc_ros/<robot name>/right_foot_frame_name`

For examples, see the launch files in `ihmc_atlas_ros` or `ihmc_valkyrie_ros`, which use a special launch file in the "common" subdirectory of the "launch" directory to configure these topics.

###Usage

- Roslaunch an SCS simulation with the IHMC ROS API node, e.g. `roslaunch ihmc_valkyrie_ros ihmc_valkyrie_scs.launch`
- `rosrun ihmc_ros_diagnostics boxStep.py`

## Arm demos

Two arm demos are provided. Similar to the box step demos, the robot name paramter must be set as well as the names of the robot's arm joints as an array in kinematic ordering. For examples of how to configure these parameters, you can look at the launch files in `ihmc_atlas_ros` or `ihmc_valkyrie_ros`.

The parameter names are:

- `/ihmc_ros/<robot name>/right_arm_joint_names`
- `/ihmc_ros/<robot name>/left_arm_joint_names`

###Usage

- Roslaunch an SCS simulation with the IHMC ROS API node, e.g. `roslaunch ihmc_valkyrie_ros ihmc_valkyrie_scs.launch`
-`rosrun ihmc_ros_diagnostics armDemo1.py` or `rosrun ihmc_ros_diagnostics armDemo2.py`
