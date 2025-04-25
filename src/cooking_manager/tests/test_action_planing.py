#!/usr/bin/env python
import rospy
import unittest
from std_msgs.msg import String
from cooking_manager.msg import RecipeStep


class TestActionPlanning(unittest.TestCase):
    @classmethod
    def setUpClass(self) -> None:
        rospy.init_node('action_planning_test_node', anonymous=True)

        self.dummy_steps=["cut the onions","cook onion","boil the water"]

        self.dummy_action_seqences=[
                                    ["grab the onions urgent", "cut the onions not_urgent"],
                                    ["get onion not_urgent", "put in pan not_urgent", "turn on low heat not_urgent"],
                                    ["grab the pan not_urgent", "fill the pan with water urgent", "put the pan on the stove urgent"]
                                    
                                    ]
        




    def setUp(self):
        self.step_publisher = rospy.Publisher('recipe_step', RecipeStep, queue_size=10)
        self.rate= rospy.Rate(10)  # 10 Hz
        self.Subscriber=rospy.Subscriber('action_sequence',String, self.sequence_callback)

        rospy.sleep(1)  # Allow some time for the publisher to register

        

    
    def sequence_callback(self,msg):
            # Convert the incoming string message to a list and append it to received_sequences
            new_seq=msg.data.split(",")
            self.received_sequences.append(new_seq)

    def test_parsing_steps(self): 
        '''
        this test checks the step parsing function of the action planning module
        '''

        self.received_sequences = []


        # Subscribe to the topic where the action sequences are published

        for step in self.dummy_steps:
            # Publish the current step
            rospy.loginfo(f"Publishing step: {step}")
            self.step_publisher.publish(step)
            

            rospy.sleep(1)
            
            self.rate.sleep()



        # Check if the received sequences match the dummy action sequences
        if len(self.received_sequences) == len(self.dummy_action_seqences):
            for received, expected in zip(self.received_sequences, self.dummy_action_seqences):
                self.assertEqual(received, expected)
        else:
            self.fail(f"Number of received sequences ({len(self.received_sequences)}) does not match expected ({len(self.dummy_action_seqences)}).")


            


        



        

        


if __name__ == "__main__":
    import rostest
    rostest.rosrun('cooking_manager', 'test_action_planning',
                   TestActionPlanning)
