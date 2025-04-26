#!/usr/bin/env python3
import rospy
import rostest
import unittest
from std_msgs.msg import String

class TestHumanCommandManager(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_human_command_manager", anonymous=True)
        self.received_cmds = []
        self.verbal_pub = rospy.Publisher("/verbal_command", String, queue_size=10)
        self.state_pub = rospy.Publisher("/system_state", String, queue_size=10)
        self.action_pub = rospy.Publisher("/current_actions", String, queue_size=10)
        rospy.sleep(1)
        rospy.Subscriber("/valid_command", String, self._cmd_callback)


    def _cmd_callback(self, msg):
        self.received_cmds.append(msg.data)
    def publish_inputs(self, state, actions, verbal_cmd):
        self.action_pub.publish(String(data=",".join(actions)))  # 1st publish actions
        rospy.sleep(0.5)
        self.state_pub.publish(String(data=state))               # 2nd publish state
        rospy.sleep(0.5)
        self.verbal_pub.publish(String(data=verbal_cmd))          # 3rd publish verbal command
        rospy.sleep(2)


    def test_valid_command_idle(self):
        self.received_cmds.clear()
        self.publish_inputs("IDLE", [], "cut orange")
        self.assertIn("cut orange", self.received_cmds)

    def test_valid_command_failure(self):
        self.received_cmds.clear()
        self.publish_inputs("FAILURE", [], "cut onion")
        self.assertNotIn("cut onion", self.received_cmds)


    def test_valid_command_executing_accepted(self):
        self.received_cmds.clear()
        actions = [
        "grab the onions not_urgent",
        "cut the onions not_urgent",
        "wait 2 seconds not_urgent",
        "get onion not_urgent",
        "put in pan not_urgent",
        "turn on low heat not_urgent",
        "grab the pan not_urgent",
        "fill the pan with water urgent",
        "put the pan on the stove urgent"
    ]

        
        self.publish_inputs("EXECUTING", actions, "cut orange")
        self.assertIn("cut orange", self.received_cmds)

  
    def test_valid_command_executing_rejected(self):
        self.received_cmds.clear()
        actions = [
            "grab the onions not_urgent", "cut the onions urgent",
            "get onion urgent", "put in pan urgent", "turn on low heat urgent",
            "grab the pan urgent", "fill the pan with water urgent", "put the pan on the stove not_urgent"]
     
        self.publish_inputs("EXECUTING", actions, "fill pot urgent")
        self.assertNotIn("fill pot urgent", self.received_cmds)

if __name__ == '__main__':
    rostest.rosrun('cooking_manager', 'test_human_command_manager', TestHumanCommandManager, sysargs=['-v'])
