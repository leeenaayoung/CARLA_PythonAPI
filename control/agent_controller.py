import carla
from enum import Enum
import time

class DriveMode(Enum):
    AUTO = 0
    MANUAL = 1

class AgentController:
    def __init__(self, agent, wheel,auto_resume_delay=2.0):
        self.agent = agent
        self.wheel = wheel

        self.mode = DriveMode.AUTO
        self.last_human_input_time = None
        self.auto_resume_delay = auto_resume_delay

    def step(self):
        if self.mode == DriveMode.MANUAL:
            if self.wheel and self.wheel.detect_intervention():
                self.last_human_input_time = time.time()
                return self.wheel.get_human_control()

            if self.last_human_input_time and \
                (time.time() - self.last_human_input_time) > self.auto_resume_delay:
                self.mode = DriveMode.AUTO
            else:
                return self.wheel.get_human_control()

        if self.mode == DriveMode.AUTO:
            if self.wheel and self.wheel.detect_intervention():
                self.mode = DriveMode.MANUAL
                self.last_human_input_time = time.time()
                return self.wheel.get_human_control()
            else:
                return self.agent.run_step()