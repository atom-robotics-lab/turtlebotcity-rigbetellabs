#! /bin/bash

source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/turtlebotcity-rigbetellabs/turtlebotcity_gazebo/models/:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/turtlebotcity-rigbetellabs/turtlebotcity_gazebo/models/:${CATKIN_ENV_HOOK_WORKSPACE}/../src/turtlebotcity-rigbetellabs/turtlebotcity_gazebo/worlds/:${GAZEBO_RESOURCE_PATH}"
export TURTLEBOT3_MODEL=burger