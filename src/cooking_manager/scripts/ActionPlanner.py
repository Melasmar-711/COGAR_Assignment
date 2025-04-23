
class ActionPlanner:
    def __init__(self):
      

    # One function to receive all inputs (e.g., from subscribers or services)
    def receive_inputs(self, current_step, valid_command):
        self.current_step = current_step
        self.valid_command = valid_command

    # One function to decide and send/publish the current action
    def currnet_action(self):
        # Logic goes here (e.g., choose between step and command)
        return self.current_action
