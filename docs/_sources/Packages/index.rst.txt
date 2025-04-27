Packages Documentation
======================

.. contents:: 
   :local:
   :depth: 1

Overview of Packages
--------------------
This section documents all ROS packages comprising the system. Below is a summary of their roles and relationships:

+---------------------+-------------------------------------------------------+-----------------------------------------+
| Package Name        | Description                                           | Key Components                          |
+=====================+=======================================================+=========================================+
| `controller`_       | Manages system state and coordinates action execution | - Action Parser Node                    |
|                     |                                                       | - trajectory_manage node                |
|                     |                                                       | - PID controller node                   |
+---------------------+-------------------------------------------------------+-----------------------------------------+
| `perception`_       | Handles sensor processing and object detection        | - SLAM                                  |
|                     |                                                       | - Sensor Fusion                         |
|		      | 						      | - computer vision		 	|
+---------------------+-------------------------------------------------------+-----------------------------------------+
| `cooking_manager`_  | Manages recipe execution and action sequencing        | - Recipe tracker                        |
|                     |                                                       | - Action planner	                |
|		      |						              |-humman command monitoring		|
+---------------------+-------------------------------------------------------+-----------------------------------------+

Package Details
---------------
.. toctree::
   :maxdepth: 2
   :caption: Individual Package Documentation:
   
   controller
   perception
   cooking_manager
   

-------------------------------------



