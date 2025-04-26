source devel/setup.sh
rosrun cooking_manager recipe_tracker.py &
rosrun cooking_manager action_planner.py &
rosrun cooking_manager human_command_manager.py 