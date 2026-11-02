"""
command_script.py
=================
Plays a list of robot-window commands automatically — useful for demos
("run lesson 3 by itself") and for automated checks of the whole example.

Enable it with the controller argument  --script=path/to/script.json
The file is a JSON list of steps, executed one after another:

    {"wait": "IDLE"}                                  wait until the controller is in IDLE
    {"wait": "IDLE", "send": {"cmd": "launch", ...}}  …then send a window command
    {"wait": "PAUSED:APPROACH", "delay": 0.5,         teaching-mode pause before APPROACH,
     "send": {"cmd": "next"}}                         held for 0.5 s, then "Next phase"
    {"wait": "CAUGHT", "timeout": 30}                 fail if not reached within 30 s

When every step has run, Webots quits with exit code 0; if a step times
out it prints why and quits with exit code 1.
"""

import json


class CommandScript:

    def __init__(self, path):
        with open(path) as f:
            self.steps = json.load(f)
        self.step_start = None
        self.reached_at = None
        self.index = 0

    def update(self, demo):
        """Call once per control step.  Returns False when the script failed."""
        if self.index > len(self.steps):
            return True                          # finished (or failed) earlier
        if self.index == len(self.steps):
            print(f"SCRIPT PASSED: all {len(self.steps)} steps")
            demo.quit_later(0)
            self.index += 1                      # print and quit only once
            return True

        now = demo.robot.getTime()
        step = self.steps[self.index]
        if self.step_start is None:
            self.step_start = now

        state = demo.state
        if state == "PAUSED":
            state = f"PAUSED:{demo.paused_next}"
        if state == step["wait"]:
            if self.reached_at is None:
                self.reached_at = now
            if now - self.reached_at >= step.get("delay", 0.0):
                print(f"SCRIPT step {self.index + 1}: reached {step['wait']} at t={now:.2f} s"
                      + (f", sending {step['send']}" if "send" in step else ""))
                if "send" in step:
                    demo.handle_message(json.dumps(step["send"]))
                self.index += 1
                self.step_start = self.reached_at = None
        else:
            self.reached_at = None
            if now - self.step_start > step.get("timeout", 30.0):
                print(f"SCRIPT FAILED at step {self.index + 1}: waited for {step['wait']}, "
                      f"state is {state}")
                demo.quit_later(1)
                self.index = len(self.steps) + 1
                return False
        return True
