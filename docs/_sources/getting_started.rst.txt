Getting Started
===============

Prerequisites
-------------
- ROS Noetic
- Ubuntu 20.04
- Python 3.8

Installation
------------
.. code-block:: bash

   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash

Running the System
------------------
.. code-block:: bash

   ./launch_system.sh
   ./play_bags.sh
   
   
Running unit tests
------------------
.. code-block:: bash 
   
   rostest cooking_manager test_action_planing.py
   rostest cooking_manager test_human_command_manager
   rostest cooking_manager test_recipe_tracking
   
Running Integration Tests
-------------------------
.. code-block:: bash 
   
   rostest cooking_manager test_integration_tests.py

