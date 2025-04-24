#!/usr/bin/env python3
import rospy
import unittest
import json
from controller.msg import SystemState
from scripts.recipe_tracker import RecipeTracker
from cooking_manager.msg import RecipeStep
from cooking_manager.srv import UpdateRecipe, UpdateRecipeResponse  

class TestRecipeTracker(unittest.TestCase):
    @classmethod
    def setUpClass(self) -> None:
        rospy.init_node('recipe_tracker_test_node', anonymous=True)
        self.recipe_tracker = RecipeTracker()
        self.current_step = None
        self.dummy_recipe = {
            "steps": [
                "pick the pan and put it on the stove",
                "add oil on the pan"
            ]
        }
        self.dummy_recipe_text = json.dumps(self.dummy_recipe)
        self.update_recipe_service_proxy = rospy.ServiceProxy('update_recipe', UpdateRecipe)
        self.idle_system_state = SystemState("IDLE")
        self.executing_system_state = SystemState("EXECUTING")
        self.failure_system_state = SystemState("FAILURE")

        
        self.system_state_pub = rospy.Publisher("system_state", SystemState)
        rospy.wait_for_service("update_recipe")
    
    def hear_recipe_step(self,recipe_step:RecipeStep):
        self.current_step = recipe_step

    def setUp(self):
        # self.recipe_tracker = RecipeTracker()
        rospy.Subscriber("recipe_step", RecipeStep, self.hear_recipe_step)
        rospy.sleep(1)
    

    
    # def test_parse_recipe(self):

    #     recipe_parsed = self.recipe_tracker.parse_recipe(self.dummy_recipe_text)

    #     self.assertIsNotNone(recipe_parsed, "Recipe parsing returned None")
    #     self.assertEqual(len(recipe_parsed["steps"]), 2, "Incorrect number of steps parsed")
    #     self.assertEqual(recipe_parsed["steps"][0], self.dummy_recipe["steps"][0], 
    #                     "First step doesn't match")
    #     self.assertEqual(recipe_parsed["steps"][1], self.dummy_recipe["steps"][1], 
	# 	                "Second step doesn't match")
    
    def test_recipe_tracker_when_idle(self):
        result = self.update_recipe_service_proxy(self.dummy_recipe_text)
        self.assertTrue(result.success,"error while calling the service")

        self.system_state_pub.publish(self.idle_system_state)
        rospy.sleep(3.5)
        self.system_state_pub.publish(self.idle_system_state)
        rospy.sleep(1.0)

        self.assertEqual(self.current_step, RecipeStep(self.dummy_recipe["steps"][1]), 
		                "Step is not valid")
        
    def test_recipe_tracker_when_execution(self):
        result = self.update_recipe_service_proxy(self.dummy_recipe_text)
        self.assertTrue(result.success,"error while calling the service")

        self.system_state_pub.publish(self.executing_system_state)
        rospy.sleep(3.5)
        self.system_state_pub.publish(self.executing_system_state)
        rospy.sleep(1.0)

        self.assertEqual(self.current_step, RecipeStep(self.dummy_recipe["steps"][0]), 
		                "Step is not valid")
    
    def test_recipe_tracker_when_failure(self):
        result = self.update_recipe_service_proxy(self.dummy_recipe_text)
        self.assertTrue(result.success,"error while calling the service")

        self.system_state_pub.publish(self.failure_system_state)
        rospy.sleep(3.5)
        self.system_state_pub.publish(self.failure_system_state)
        rospy.sleep(1.0)

        self.assertEqual(self.current_step, RecipeStep(self.dummy_recipe["steps"][0]), 
		                "Step is not valid")
    


if __name__ == '__main__':
    import rostest
    rostest.rosrun('cooking_manager', 'test_recipe_tracker', TestRecipeTracker)
		