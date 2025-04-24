#!/usr/bin/env python3
import rospy
import unittest
import json
# from cooking_manager.srv import RecipeText
from controller.msg import SystemState
from scripts.recipe_tracker import RecipeTracker

class TestRecipeTracker(unittest.TestCase):
    @classmethod
    def setUpClass(self) -> None:
        self.recipe_tracker = RecipeTracker()
        self.dummy_recipe = {
            "steps": [
                "pick the pan and put it on the stove",
                "add oil on the pan"
            ]
        }
        self.dummy_recipe_text = json.dumps(self.dummy_recipe)
        # cls.recipe_text_service_proxy = rospy.ServiceProxy('recipe_text', RecipeText)
        self.idle_system_state = SystemState("IDLE")
        self.executing_system_state = SystemState("EXECUTING")
        self.failure_system_state = SystemState("FAILURE")
        # rospy.wait_for_service("recipe_text")
    def setUp(self):
        self.recipe_tracker = RecipeTracker()

    def test_parse_recipe(self):

        recipe_parsed = self.recipe_tracker.parse_recipe(self.dummy_recipe_text)

        self.assertIsNotNone(recipe_parsed, "Recipe parsing returned None")
        self.assertEqual(len(recipe_parsed.steps), 2, "Incorrect number of steps parsed")
        self.assertEqual(recipe_parsed.steps[0], self.dummy_recipe["steps"][0], 
                        "First step doesn't match")
        self.assertEqual(recipe_parsed.steps[1], self.dummy_recipe["steps"][1], 
		                "Second step doesn't match")
        
    def test_recipe_tracker_when_idle(self):
        self.recipe_tracker.update_progress(self.idle_system_state)

        self.assertEqual(self.recipe_tracker.current_step(), self.dummy_recipe["steps"][1], 
		                "Step is not valid")
        
    def test_recipe_tracker_when_execution(self):
        self.recipe_tracker.update_progress(self.executing_system_state)

        self.assertEqual(self.recipe_tracker.current_step(), self.dummy_recipe["steps"][0], 
		                "Step is not valid")
    
    def test_recipe_tracker_when_failure(self):
        self.recipe_tracker.update_progress(self.failure_system_state)

        self.assertEqual(self.recipe_tracker.current_step(), self.dummy_recipe["steps"][0], 
		                "Step is not valid")
    


if __name__ == '__main__':
    import rostest
    rostest.rosrun('cooking_manager', 'test_recipe_tracker', TestRecipeTracker)
		