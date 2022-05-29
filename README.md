# SOFAR assignment

## prerequisites 

This simulation works on the ROS galactic distribution.

To use this package you need to first install the `nav2_bringup`, `navigation2`and `slam_toolbox`packages.

`sudo apt-get install 

## installation 

Add these packages into the src directory of your ros workspace.

update the dependencies using :

`rosdep update`
`rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO`

Compile your workspace.
You can then run the simulation using the command

`ros2 launch sofar_tiago sofar_launch.py`

### Launch arguments

These are the launch arguments of the launch file : 
- `slam` : use slam mapping  (default : False)
- `nav` : use navigation (default : False)
- `rviz` : use rviz (default : False)
- `use_sim_time` : use the simulation time(default : True)
- `world`: choose the current 3D world (default : `default.wbt` file in the worlds directory)
- `mode`: choose the simulator mode (default : 'realtime')
- `map`: choose the map used for navigation ( default : `map.yaml` file in the resources directory)
- `slam_params`:choose the parameters for slam mapping (default : `slam_params.yaml` file in the params directory)
- `nav_params`:choose the parameters for navigation (default : `navigation_params.yaml` file in the params directory)

## slam launch

Launch the simulation using the command 

`ros2 launch sofar_tiago  sofar_launch.py slam:=true rviz:=true`

To move the robot and create the map, use 

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

and then control the robot using your keyboard.

To save the map, use

`ros2 run nav2_map_server map_saver_cli -f ~/map`

## navigation launch

To run the robot with the navigation stack, run 

`ros2 launch sofar_tiago  sofar_launch.py nav:=true rviz:=true`

Then place the robot using a `2D pose Estimate` and create a goal using a `2D Nav Goal`.


