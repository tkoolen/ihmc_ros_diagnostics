#ihmc_diagnostics

Diagnostics tools for analyzing the IHMC ROS distribution

## Box Step demo

The Box Step demo is a simple test/example of commanding the robot using Footstep Lists from `ihmc_msgs`. It can be used as a pattern matching example and also to vet the walking and balancing algorithm on hardware.

###Usage
`rosrun ihmc_diagnostics boxStep.py`

## Automated Controller Diagnostics

The automated controller diagnostics are a more thorough workout of the whole body controller.  This can be run in simulation or on the real robot.

###Usage
`roslaunch ihmc_diagnostics ihmc_sim_diagnostic.launch`: Run the automated diagnostic behavior in sim. Useful to see what the exercises look like.

`roslaunch ihmc_diagnostics ihmc_atlas_diagnostic.launch`: Run the automated diagnostic behavior on the real robot. Useful for verifying the robustness of the control algorithm on different hardware. **(Coming Soon)**

`roslaunch ihmc_diagnostics ihmc_atlas_visualizer.launch`: Bring up the Remote Atlas Visualizer; this is a remote Simulation Construction Set instance that receives data from the Atlas robot, allowing for introspection in to the state of the controller during debugging. *The visualizer is a debugging tool and should not be run on the DRC competition network*, as it is not bandwidth limited. **(Coming Soon)**
