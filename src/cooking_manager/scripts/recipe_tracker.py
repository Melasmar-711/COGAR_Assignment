#!/usr/bin/env python3

from cooking_manager.msg import RecipeStep
from controller.msg import SystemState
from typing import List, Optional
import rospy

class RecipeTracker:
    
    def __init__(self):
        pass
    # Parses raw recipe text and initializes the internal structure
    def parse_recipe(self, raw_text: str) -> List[RecipeStep]:
        
        return None

    # Stores recipe internally
    def store_recipe(self, steps: List[RecipeStep]) -> None:
        return None
      

    # Returns the current recipe step
    def current_step(self) -> Optional[RecipeStep]:
       
        return None

    # Updates the progress based on external system state
    def update_progress(self, system_state: SystemState) -> None:
        
        return None
        

    # Checks if the system is executing or idle (True = executing, False = done)
    def recipe_state(self) -> bool:
        return None

    # Returns the full recipe for debug or query purposes
    def get_full_recipe(self) -> List[RecipeStep]:
        return None

if __name__ == "__main__":
    rospy.init_node('add_two_ints_server')
    rospy.spin()