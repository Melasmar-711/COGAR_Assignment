Controller Package
==================

.. contents::
   :local:
   :depth: 2

Package Overview
----------------
The controller package manages system state and motion execution.


Nodes
-----
Action Parser
~~~~~~~~~~~~~~~~~~
.. automodule:: controller.scripts.action_parser
   :members:
   :undoc-members:

   **ROS Interfaces:**
   
   Service Clients:
     - ``/send_set_points`` → :class:`assignments.srv.CheckJointState` (Sends joint commands to Trajectory Manager)
   
   Subscribers:
     - ``/objects_of_interest`` (:class:`perception.msg.DetectedObjects`)
   
   Publishers:
     - ``/system_state`` (:class:`controller.msg.SystemState`)
     - ``/action_index`` (:class:`std_msgs.msg.Int32`)
     


Trajectory Manager
~~~~~~~~~~~~~~~~~~
.. automodule:: controller.scripts.trajectory_manager
   :members:
   :undoc-members:
   :show-inheritance:

   **ROS Interfaces:**
   
   Services:
     - ``/get_trajectory`` (:class:`assignments.srv.CheckJointState`)
     
 
PID Controller
~~~~~~~~~~~~~~
.. automodule:: controller.scripts.PID_Controller
   :members:
   :undoc-members:
   :show-inheritance:


Messages
--------


System State
~~~~~~~~~~~~
.. autoclass:: controller.msg.SystemState
   :members:
   :inherited-members:
