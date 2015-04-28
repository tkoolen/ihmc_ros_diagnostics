#ihmc\_diagnostics

Diagnostics tools for analyzing the IHMC ROS distribution

## Logging

The IHMC Controller is capable of generating custom logs that capture time-series for nearly every variable on the running robot as well as the IHMC control algorithm (tens of thousands of variables hundreds of times a second). We run this software every single time we turn the robot on; it is not required of you to do so, but it is useful in diagnosing bugs you may encounter.

###Usage

`roslaunch ihmc_diagnostics ihmc_logger.launch`: Run the IHMC Logging software for logging data from the controller.

## Box Step demo

The Box Step demo is a simple test/example of commanding the robot using Footstep Lists from `ihmc_msgs`. It can be used as a pattern matching example and also to vet the walking and balancing algorithm on hardware.

###Usage
`rosrun ihmc_diagnostics boxStep.py`

## Automated Controller Diagnostics

The automated controller diagnostics are a more thorough workout of the whole body controller.  This can be run in simulation or on the real robot.

###Usage

`roslaunch ihmc_diagnostics ihmc_sim_diagnostic.launch`: Run the automated diagnostic behavior in sim using SCS. Useful to see what the exercises look like.

`roslaunch ihmc_diagnostics ihmc_atlas_diagnostic.launch`: Run the automated diagnostic behavior on the real robot. Useful for verifying the robustness of the control algorithm on different hardware. **(Coming Soon)**
