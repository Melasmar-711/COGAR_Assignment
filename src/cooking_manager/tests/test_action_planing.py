#!/usr/bin/env python
import rospy
import unittest
import random


class TestActionPlanning(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node('test_action_planning', anonymous=True)

        

    def test_parsing_step(self): 
    	
    	dummy_steps=["cut the onions","boil the water"]
        


if __name__ == "__main__":
    import rostest
    rostest.rosrun('example_pkg', 'test_add_two_ints_server',
                   TestAddTwoIntsServer)
