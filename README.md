# Testing Model Views

The enviroment variable will be set automatically to source the models for the gazebo simlation

> `env_hooks` won't work with `zsh`



## RUN Gazebo Simulation
```shell
roslaunch turtlebotcity_gazebo turtlecity.launch 
```

## RUN Turtlebot Navigation Stack

You can run the turtlebot navigation stack using the below command.
The same launch file can be used to launch the navigation stack for the real robot as well.

```shell
roslaunch turtlebot3_navigation turtlebot3_navigation.launch 
```


## Optional (RUN Custom)Navigation Stack
You can also run the custon Navigation stack instead of the turtlebot3 navigation stack using the command below.
```shell
roslaunch turtlebotcity_maps navigation.launch 
```

## Run the LaneFollower Script
Use the command given below to run the lane detection and controller script/
```shell
rosrun turtlebotcity_maps LaneFollower.py
```

## Run the Image Viewer
To run the image viewer, use the command below to launch the rqt image viewer and select appropriate topic.
```shell
rosrun rqt_image_view rqt_image_view
```
