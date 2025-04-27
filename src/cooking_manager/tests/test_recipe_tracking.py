#!/usr/bin/env python3
import rospy
import unittest
import json
from controller.msg import SystemState
from cooking_manager.recipe_tracker import RecipeTracker
from cooking_manager.msg import RecipeStep
from cooking_manager.srv import UpdateRecipe, UpdateRecipeResponse  

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
		
