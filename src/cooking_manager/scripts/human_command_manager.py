#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class HumanCommandManager:
    def __init__(self):
        self._last_verbal_cmd = None
        self._current_state = "IDLE"
        self.currnet_actions_sequence = []
        self._has_published = False

        # Subscribers
        rospy.Subscriber("/verbal_command", String, self.verbal_callback)
        rospy.Subscriber("/system_state", String, self.state_callback)
        rospy.Subscriber("/current_actions", String, self.actions_callback)

        # Publisher
        self._valid_pub = rospy.Publisher("/valid_command", String, queue_size=1)

    def verbal_callback(self, msg):
        self._last_verbal_cmd = msg.data.strip()
        self._has_published = False
        rospy.loginfo(f"[RECEIVED] Verbal command: {self._last_verbal_cmd}")
        self.evaluate_and_publish()

    def state_callback(self, msg):
        self._current_state = msg.data.upper()
        rospy.loginfo(f"[STATE] Updated: {self._current_state}")
        self.evaluate_and_publish()

    def actions_callback(self, msg):
        self.currnet_actions_sequence = [action.strip() for action in msg.data.split(',') if action.strip()]
        rospy.loginfo(f"[ACTIONS] Updated: {self.currnet_actions_sequence}")
        self.evaluate_and_publish()

    def evaluate_and_publish(self):
        # Don't do anything if no new verbal command
        if not self._last_verbal_cmd or self._has_published:
            return

        if self._current_state == "IDLE":
            self.publish_valid_command(self._last_verbal_cmd)

        elif self._current_state == "EXECUTING":
            if self.is_non_urgent_ratio_low():
                self.publish_valid_command(self._last_verbal_cmd)
            else:
                rospy.loginfo("[REJECTED] Too many non-urgent actions. Command dropped.")

        elif self._current_state == "FAILURE":
            rospy.loginfo("[REJECTED] System in FAILURE. Command dropped.")

    def is_non_urgent_ratio_low(self):
        if not self.currnet_actions_sequence:
            return True  # If no actions, treat as low non-urgent

        non_urgent_count = sum(1 for action in self.currnet_actions_sequence if "not_urgent" in action)
        ratio = non_urgent_count / len(self.currnet_actions_sequence)
        rospy.loginfo(f"[EVAL] Non-urgent ratio: {ratio:.2f}")
        return ratio > 0.5  

    def publish_valid_command(self, command):
        self._valid_pub.publish(String(data=command))
        self._has_published = True
        rospy.loginfo(f"[PUBLISHED] Valid Command: {command}")

if __name__ == "__main__":
    rospy.init_node("human_command_manager")
    manager = HumanCommandManager()
    rospy.spin()

