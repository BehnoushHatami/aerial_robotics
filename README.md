# Package Delivery Mission 

The report and simulation video can be found [here](https://drive.google.com/drive/folders/1Dexa_s3G5taOIP_MUfnymewpamH5Bdi4) 

To run the whole package delivery mission, please run the following commands sequentially:

### T1: source from `ardupilot@arif:~/aerial_robotics_ws$` 
```
source devel/setup.bash

``` 

if we don't run `roscore` command then gazebo master would be commanded 

### T1: [Launch ROS Gazebo SIM in new terminal]:
```
roslaunch robowork_minihawk_gazebo minihawk_playpen.launch
```

### T2: [Launch Ardupilot Gazebo SITL in new terminal]:
```
cd $HOME/aerial_robotics_ws/ardupilot
```
```
./Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-minihawk --model gazebo-quadplane-tilttri --console  # --map
```

### T3:[Launch Rviz in new terminal]:
```
rviz -d $HOME/aerial_robotics_ws/src/aerial_robotics/robowork_minihawk_launch/config/minihawk_SIM.rviz
```

## MAVProxy-based commanding ##

### T2:[Load sample mission waypoints in Gazebo SITL terminal]
```
wp load ../src/aerial_robotics/robowork_minihawk_gazebo/resources/waypoints.txt
```

### MAVROS-based commanding ###

### T1:[Launch ROS node in new terminal 1]:
Don't forget to source the workspace.
```
ROS_NAMESPACE="minihawk_SIM" roslaunch robowork_minihawk_launch vehicle1_apm_SIM.launch
```

### T2:[Invoke ROS services in new terminal 2]:
These commands lift the drone to 10 meters in GUIDED mode. For the actual mission, we need to switch to AUTO mode, which command is provided in the next cell.
```
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'GUIDED'"
rosservice call /minihawk_SIM/mavros/cmd/arming True
rosservice call /minihawk_SIM/mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}"
```


### T2:[Alternatively, invoke ROS services in new terminal 2 to run auto (waypoint) mission]:
```
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'AUTO'"
```
```
rosservice call /minihawk_SIM/mavros/cmd/arming True   ###Required if the mission hasn't started yet### 
```

### T3: [Run `apriltag` package for precision landing] 

First source the bash file
```
cd ~$HOME/aerial_robotics_ws
source devel/setup.bash
```

To run `apriltag_detector_node` from `apriltag` ros package: 

```
rosrun apriltag tag_data.py
```
