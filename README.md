# SOFAR assignment

## prerequisites 

This simulation works on the ROS galactic distribution.

To use this package you need to first install the `nav2_bringup`, `navigation2`and `slam_toolbox`packages.

`sudo apt-get install ros-galactic-slam-toolbox`

`sudo apt-get install ros-galactic-navigation2`

`sudo apt-get install ros-galactic-nav2-bringup`


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
- `localization_only` : use slam_toolbox in localization mode (defalut : False)
- `use_sim_time` : use the simulation time(default : True)
- `world`: choose the current 3D world (default : `default.wbt` file in the worlds directory, can be replaced with `default1.wbt` or `default2.wbt`)
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

You can alternatively use nav2 goals to control the robot. 

To save the map, use

`ros2 run nav2_map_server map_saver_cli -f ~/map`

To serialize the map in posegraph format, add the slam_toolbox pannel in rviz and click on the "serialize map" button. You can also use this pannel to save a map in yaml format.  



https://user-images.githubusercontent.com/69837845/174390819-83c75a17-1627-4436-84c7-cd992dd0aa57.mp4



## navigation launch

To run the robot with the navigation stack, run 

`ros2 launch sofar_tiago  sofar_launch.py nav:=true rviz:=true`

Then place the robot using a `2D pose Estimate` and create a goal using a `2D Nav Goal`.  


https://user-images.githubusercontent.com/69837845/174393153-39d8054e-76cc-4d69-89be-7fd954d2678f.mp4



## localization only mode 

Once a map has been serialized (i.e. in posegraph format), you MUST add its full path on the "slam_params_localization.yaml" file, and run 

 `ros2 launch sofar_tiago  sofar_launch.py localization_only:=true rviz:=true`
 
 This will allow you to use the slam_toolbox package as localization only using your map file. 
 
 A test file can be used in the /maps folder of the sofar_tiago package  
 

https://user-images.githubusercontent.com/69837845/174442485-c8989bb1-29e0-48ab-ae14-dd5b0d39945e.mp4
 

## Continue mapping mode

If you want to map an environment in several sessions, you can use the continue mapping mode. At the end of one session, you should save the map and its pose graph. In mapper_params_lifelong, the map_file_name (line 33) should be updated for the map name. You can then launch the simulation with:

`ros2 launch sofar_tiago  sofar_launch.py continue:=true rviz:=true`

In rviz, through the slam toolbox plugin, you can then deserialize the pose graph and continue to map the environment.
As of right now, this mode is experimental and we did not manage to calibrate it well, the mapping is not accurate at all.
 
https://user-images.githubusercontent.com/69837845/174442839-3877486b-6fde-437f-983a-3c0bfb05f325.mp4
