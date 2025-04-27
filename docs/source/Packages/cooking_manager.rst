Cooking Manager Package
=======================

.. contents::
   :local:
   :depth: 2

Package Overview
----------------
The controller manages the recipe excecution and humman command interpretation


Nodes
-----

Recipe Tracker
~~~~~~~~~~~~~~~~~~
.. automodule:: scripts.recipe_tracker
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



Human Command Manager
~~~~~~~~~~~~~~~~~~~~~
.. automodule:: scripts.human_command_manager
   :members:
   :undoc-members:
   :show-inheritance:



     
     

Action Planner
~~~~~~~~~~~~~~
.. automodule:: scripts.action_planner
   :members:
   :undoc-members:
   :show-inheritance:




Messages
--------


System State
~~~~~~~~~~~~
.. autoclass:: cooking_manager.msg.RecipeStep
   :members:
   :inherited-members: 
   

Services   
--------


Send Action Sequence
~~~~~~~~~~~~~~~~~~~~
.. autoclass:: cooking_manager.srv.SendActionSeq
   :members:
   :show-inheritance:   
   

Update Recipe
~~~~~~~~~~~~~
.. autoclass:: cooking_manager.srv.UpdateRecipe
   :members:
   :show-inheritance:  
   


Unit Tests
----------


   
