#!/bin/bash

# Kill any existing ROS processes
killall -9 roscore
killall -9 rosrun

# Start roscore in first terminal (top-left)
gnome-terminal --geometry 80x24+0+0 -- bash -c "source devel/setup.sh; roscore"

# Cooking Manager nodes (top section)
gnome-terminal --geometry 80x24+800+0 -- bash -c "source devel/setup.sh; rosrun cooking_manager recipe_tracker.py"
gnome-terminal --geometry 80x24+0+300 -- bash -c "source devel/setup.sh; rosrun cooking_manager action_planner.py"
gnome-terminal --geometry 80x24+800+300 -- bash -c "source devel/setup.sh; rosrun cooking_manager human_command_manager.py"

# Controller nodes (middle section)
gnome-terminal --geometry 80x24+0+600 -- bash -c "source devel/setup.sh; rosrun controller action_parser.py"
gnome-terminal --geometry 80x24+800+600 -- bash -c "source devel/setup.sh; rosrun controller PID_Controller.py"
gnome-terminal --geometry 80x24+0+900 -- bash -c "source devel/setup.sh; rosrun controller tracjectory_manager.py"

# Other nodes (bottom section)
gnome-terminal --geometry 80x24+800+900 -- bash -c "source devel/setup.sh; rosrun assignments arm_motion_service.py"
gnome-terminal --geometry 80x24+0+1200 -- bash -c "source devel/setup.sh; rosrun perception computer_vision.py"
gnome-terminal --geometry 80x24+800+1200 -- bash -c "source devel/setup.sh; rosrun perception sensor_fusion.py"
gnome-terminal --geometry 80x24+400+1200 -- bash -c "source devel/setup.sh; rosrun perception slam.py"
