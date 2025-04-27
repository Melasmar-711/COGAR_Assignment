#!/bin/bash

# Kill any existing ROS processes
killall -9 roscore
killall -9 rosrun

# Start roscore in first terminal (top-left)
source devel/setup.sh

# Cooking Manager nodes (top section)
rosrun cooking_manager recipe_tracker.py &
rosrun cooking_manager action_planner.py &
rosrun cooking_manager human_command_manager.py &

rosrun controller action_parser.py &
rosrun controller PID_Controller.py &
rosrun controller tracjectory_manager.py &

rosrun assignments arm_motion_service.py &
rosrun perception computer_vision.py &
rosrun perception sensor_fusion.py &
rosrun perception slam.py
