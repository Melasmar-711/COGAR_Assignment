#!/usr/bin/env python3
"""Tracks and manages cooking recipe steps."""

from cooking_manager.msg import RecipeStep
from controller.msg import SystemState
from cooking_manager.srv import UpdateRecipe, UpdateRecipeResponse  
import rospy
import json

class RecipeTracker:
    """Keeps track of the current recipe step and progress."""
    
    def __init__(self):
        """Sets up the recipe tracker with default values."""
        self.recipe = None
        self.curr_step_idx = 0
        self.system_state = None
        self.system_idle_start_time = None
        self.was_idle = False

        self.step_pub = rospy.Publisher("recipe_step", RecipeStep, queue_size=10)
        rospy.Subscriber("system_state", SystemState, self.update_system_state)
        self.timer = rospy.Timer(rospy.Duration(0.3), self.publish_current_step)
        self.recipe_srv = rospy.Service("update_recipe", UpdateRecipe, self.handle_update_recipe)
    
    def update_system_state(self, system_state):
        """Updates the current system state."""
        self.system_state = system_state
        self.update_progress()

    def handle_update_recipe(self, req):
        """Handles requests to update the current recipe."""
        try:
            self.recipe = json.loads(req.recipe_json)
            self.curr_step_idx = 0
            return UpdateRecipeResponse(True)
        except:
            return UpdateRecipeResponse(False)

    def publish_current_step(self, event):
        """Publishes the current recipe step."""
        if self.recipe and self.curr_step_idx < len(self.recipe["steps"]):
            step = RecipeStep(self.recipe["steps"][self.curr_step_idx])
            self.step_pub.publish(step)

    def update_progress(self):
        """Updates progress based on system state."""
        if not self.system_state or not self.recipe:
            return
            
        if self.system_state.state == "IDLE" and not self.was_idle:
            if not self.system_idle_start_time:
                self.system_idle_start_time = rospy.get_time()
            elif (rospy.get_time() - self.system_idle_start_time) > 1:
                self.curr_step_idx += 1
                self.was_idle = True    
        elif self.system_state.state == "EXECUTING":
            self.system_idle_start_time = None
            self.was_idle = False
        elif self.system_state.state == "FAILURE":
            self.system_idle_start_time = None
            self.was_idle = False

if __name__ == "__main__":
    rospy.init_node('recipe_tracker')
    rospy.loginfo("Recipe Tracker Node Started")
    tracker = RecipeTracker()
    rospy.spin()
