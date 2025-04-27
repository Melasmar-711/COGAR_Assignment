#!/usr/bin/env python3

from cooking_manager.msg import RecipeStep
from controller.msg import SystemState
from typing import List, Optional
from cooking_manager.srv import UpdateRecipe, UpdateRecipeResponse  
import rospy
import json

class RecipeTracker:
    
    def __init__(self):
        self.recipe = None
        self.curr_step_idx = 0
        self.system_state = None
        self.system_idle_start_time = None
        self.was_idle = False

        self.step_pub = rospy.Publisher("recipe_step", RecipeStep, queue_size=10)
        rospy.Subscriber("system_state", SystemState, self.update_system_state)
        self.timer = rospy.Timer(rospy.Duration(0.3), self.publish_current_step)
        self.recipe_srv = rospy.Service("update_recipe", UpdateRecipe, self.handle_update_recipe)
    
    def update_system_state(self,system_state: SystemState) -> None:
        self.system_state = system_state
        self.update_progress()

    def handle_update_recipe(self, req):
        update_recipe_state = self.update_recipe(req.recipe_json)
        return UpdateRecipeResponse(success=update_recipe_state)

    def publish_current_step(self, event):
        if self.is_recipe_step_available():
            step = self.current_step()
            if step:
                self.step_pub.publish(step)

    # Parses raw recipe text and initializes the internal structure
    def parse_recipe(self, raw_text: str) -> dict:
        # try:
            recipe = json.loads(raw_text)
            return recipe
        # except:
        #     return None
        
    # Stores recipe internally
    def update_recipe(self, raw_text: str) -> bool:
        recipe = self.parse_recipe(raw_text)
        if recipe is None:
            return False
        self.recipe = recipe
        self.curr_step_idx = 0
        return True


    # Returns the current recipe step
    def current_step(self) -> Optional[RecipeStep]:
        return RecipeStep(self.recipe["steps"][self.curr_step_idx])

    # Updates the progress based on external system state
    def update_progress(self) -> None:
        if self.system_state is None or self.recipe is None:
            return
        # rospy.loginfo(f"curr_step_idx {self.curr_step_idx}")
        
        if self.system_state.state == "IDLE" and not self.was_idle:
            if self.system_idle_start_time is None:
                self.system_idle_start_time = rospy.get_time()
            elif (rospy.get_time() - self.system_idle_start_time) > 1:
                # rospy.loginfo(f"self.system_idle_start_time {self.system_idle_start_time}, time now {rospy.get_time()}, diff {(rospy.get_time() - self.system_idle_start_time)}")
                self.curr_step_idx += 1
                self.was_idle = True    

            
        elif self.system_state.state == "EXECUTING":
            self.system_idle_start_time = None
            self.was_idle = False
            print("System executing, no update")
        elif self.system_state.state == "FAILURE":
            self.system_idle_start_time = None
            self.was_idle = False
            print("System dailure, no update")
        
    
    
    def is_recipe_step_available(self) -> bool:
        return self.recipe is not None and self.curr_step_idx < len(self.recipe["steps"])

    # Returns the full recipe for debug or query purposes
    def get_full_recipe(self) -> List[RecipeStep]:
        return self.recipe

if __name__ == "__main__":
    rospy.init_node('recipe_tracker', log_level=rospy.DEBUG)
    rospy.loginfo("Recipe Tracker Node Started")
    tracker = RecipeTracker()
    rospy.spin()