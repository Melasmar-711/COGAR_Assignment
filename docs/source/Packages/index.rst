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
| controller          | Manages system state and coordinates action execution | - Action Parser Node                    |
|                     |                                                       | - trajectory_manager node               |
|                     |                                                       | - PID controller node                   |
+---------------------+-------------------------------------------------------+-----------------------------------------+
| perception          | Handles sensor processing and object detection        | - SLAM                                  |
|                     |                                                       | - Sensor Fusion                         |
|                     |                                                       | - Computer Vision                       |
+---------------------+-------------------------------------------------------+-----------------------------------------+
| cooking_manager     | Manages recipe execution and action sequencing        | - Recipe tracker                        |
|                     |                                                       | - Action planner                        |
|                     |                                                       | - Human command monitoring              |
+---------------------+-------------------------------------------------------+-----------------------------------------+

Package Details
---------------
.. toctree::
   :maxdepth: 2
   :caption: Individual Package Documentation:
   
   controller.rst
   perception.rst
   cooking_manager.rst
