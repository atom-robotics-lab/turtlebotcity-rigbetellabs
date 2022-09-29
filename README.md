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
```shell
roslaunch turtlebotcity_maps navigation.launch 
```

