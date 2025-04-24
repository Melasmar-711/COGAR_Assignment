#!/usr/bin/env python3

from cooking_manager.msg import RecipeStep
from controller.msg import SystemState
from typing import List, Optional
import rospy
import json

class RecipeTracker:
    
    def __init__(self):
        self.recipe = None
        self.curr_step_idx = 0
        self.system_state = None
        self.recipe_state = "IDLE"
    
    # Parses raw recipe text and initializes the internal structure
    def parse_recipe(self, raw_text: str) -> dict:
        recipe = json.loads(raw_text)
        return recipe
        
    # Stores recipe internally
    def update_recipe(self, raw_text: str) -> None:
        self.recipe = self.parse_recipe(raw_text)
        self.curr_step_idx = 0


    # Returns the current recipe step
    def current_step(self) -> Optional[RecipeStep]:
        return RecipeStep(self.recipe["steps"][self.curr_step_idx])

    # Updates the progress based on external system state
    def update_progress(self) -> None:
        if self.system_state.state == "IDLE":
            self.curr_step_idx += 1
            self.recipe_state =  "EXECUTING"
            if self.curr_step_idx >= len(self.recipe["steps"]):
                self.recipe_state =  "DONE"
                self.curr_step_idx = 0
                print("Recipe done")
        elif self.system_state.state == "EXECUTING":
            print("System executing, no update")
        elif self.system_state.state == "FAILURE":
            print("System dailure, no update")
        
        
    def update_system_state(self,system_state: SystemState) -> None:
        self.system_state = system_state
    
    def recipe_state(self) -> bool:
        return self.recipe_state

    # Returns the full recipe for debug or query purposes
    def get_full_recipe(self) -> List[RecipeStep]:
        return self.recipe

if __name__ == "__main__":
    rospy.init_node('add_two_ints_server')
    rospy.spin()