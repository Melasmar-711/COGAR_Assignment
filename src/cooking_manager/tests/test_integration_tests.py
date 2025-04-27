#!/usr/bin/env python
import rospy
import unittest
from std_msgs.msg import String
from cooking_manager.msg import RecipeStep
from cooking_manager.srv import SendActionSeq, SendActionSeqResponse
from std_msgs.msg import Int32
from cooking_manager.srv import UpdateRecipe, UpdateRecipeResponse  
from controller.msg import SystemState
import json



class TestIntegrationTests(unittest.TestCase):
    
    @classmethod
    def setUpClass(self) -> None:
        rospy.init_node('action_integration_test_node', anonymous=True)

        self.dummy_recipe = {
            "steps": [
                "cut the onions",
                "cook onion",
                "boil the water"
            ]
        }

        self.verbal_command_publisher = rospy.Publisher('verbal_command', String, queue_size=10)

        self.dummy_recipe_text = json.dumps(self.dummy_recipe)
        self.update_recipe_service_proxy = rospy.ServiceProxy('update_recipe', UpdateRecipe)
        self.idle_system_state = SystemState("IDLE")
        self.executing_system_state = SystemState("EXECUTING")
        self.failure_system_state = SystemState("FAILURE")
        self.logging_dict = {
            "system_state":[],
            "action_indeces":[],
        }
        rospy.wait_for_service("update_recipe")


    def setUp(self):
        rospy.Subscriber("system_state", SystemState, self.handle_system_state)
        rospy.Subscriber("action_index", Int32, self.handle_action_index)
        rospy.sleep(1)  
        
    def handle_system_state(self,msg):
        self.logging_dict["system_state"].append(msg.state)
    
    def handle_action_index(self,msg):
        if len(self.logging_dict["action_indeces"]) ==0 or self.logging_dict["action_indeces"][-1] != msg.data:
            self.logging_dict["action_indeces"].append(msg.data)
    

    # def test_recipe_success(self): 
    #     '''
    #     this test checks the step parsing function of the action planning module
    #     '''
    #     result = self.update_recipe_service_proxy(self.dummy_recipe_text)
    #     self.assertTrue(result.success,"error while calling the recipe tracker service")

    #     rospy.sleep(50)

    #     self.assertEqual(self.logging_dict["system_state"][-1],"IDLE")
    #     self.assertNotIn("FAILURE",self.logging_dict["system_state"])
    #     self.assertIn("EXECUTING",self.logging_dict["system_state"])
    #     self.assertEqual(self.logging_dict["action_indeces"],[0,1,0,1,2,0,1,2])
    
    def test_verbal_success(self): 
        '''
        this test checks the step parsing function of the action planning module
        '''


        result = self.update_recipe_service_proxy(self.dummy_recipe_text)
        self.assertTrue(result.success,"error while calling the recipe tracker service")
        rospy.sleep(1)
        self.verbal_command_publisher.publish("test verbal command")
        rospy.sleep(55)

        self.assertEqual(self.logging_dict["system_state"][-1],"IDLE")
        self.assertNotIn("FAILURE",self.logging_dict["system_state"])
        self.assertIn("EXECUTING",self.logging_dict["system_state"])
        self.assertEqual(self.logging_dict["action_indeces"],[0,1,0,1,2,0,1,2])




  


    def tearDown(self):
        self.logging_dict = {
            "system_state":[],
            "action_indeces":[],
        }
        rospy.sleep(0.5)    

            


        



        

        


if __name__ == "__main__":
    import rostest
    rostest.rosrun('cooking_manager', 'test_integration_tests',
                   TestIntegrationTests)
