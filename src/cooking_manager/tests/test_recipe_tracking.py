#!/usr/bin/env python
import rospy
import unittest
import json
from cooking_manager.srv import RecipeText
from cooking_manager.recipe_text import RecipeTracker
from controller.msg import SystemState

class TestRecipeTracker(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.recipe_tracker = RecipeTracker()
        cls.recipe_text_service_proxy = rospy.ServiceProxy('recipe_text', RecipeText)
        self.idle_system_state = SystemState("IDLE")
        self.executing_system_state = SystemState("EXECUTING")
        self.failure_system_state = SystemState("FAILURE")
        rospy.wait_for_service("recipe_text")
        
    def test_parse_recipe(self):
        test_recipe = {
            "steps": [
                "pick the pan and put it on the stove",
                "add oil on the pan"
            ]
        }

        # Convert to proper JSON string
        recipe_text = json.dumps(test_recipe)
		
        recipe_parsed = self.recipe_tracker.parse_recipe(recipe_text)

        self.assertIsNotNone(recipe_parsed, "Recipe parsing returned None")
        self.assertEqual(len(recipe_parsed.steps), 2, "Incorrect number of steps parsed")
        self.assertEqual(recipe_parsed.steps[0], test_recipe["steps"][0], 
                        "First step doesn't match")
        self.assertEqual(recipe_parsed.steps[1], test_recipe["steps"][1], 
		                "Second step doesn't match")
        
    def test_recipe_tracker_when_idle(self):
        parsed_steps = {
            "steps": 
            [
                "pick the pan and put it on the stove",
                "add oil on the pan"
            ]
        }
        self.recipe_tracker.parse_recipe(parsed_steps)
        recipe_step = self.recipe_tracker.step_tracker(self.idle_system_state)

        self.assertEqual(recipe_step, parsed_steps["steps"][1], 
		                "Second step doesn't match")
    


            
            

if __name__ == '__main__':
    import rostest
    rostest.rosrun('cooking_manager', 'test_recipe_tracker', TestRecipeTracker)
		