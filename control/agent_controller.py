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

        self.manual_by_toggle = False
        self.auto_ready = False

    def step(self, interrupt=None):
        now = time.time()

        # Check for human intervention
        if interrupt and interrupt.toggle_mode_requested:

            if self.mode == DriveMode.AUTO:
                self.mode = DriveMode.MANUAL
                self.manual_by_toggle = True
                print("[DEBUG][MODE] toggled -> DriveMode.MANUAL")
            else:
                self.mode = DriveMode.AUTO
                self.auto_ready = False
                self.manual_by_toggle = False
                print("[DEBUG][MODE] toggled -> DriveMode.AUTO")

            self.last_human_input_time = now
            interrupt.toggle_mode_requested = False
        
        if self.mode == DriveMode.MANUAL:
            # if self.wheel and self.wheel.detect_intervention():
            #     self.last_human_input_time = now
            #     return self.wheel.get_human_control()
            if self.wheel:
                return self.wheel.get_human_control()
            else:
                return carla.VehicleControl()

            # auto resume
            # if (
            #     not self.manual_by_toggle
            #     and self.last_human_input_time
            #     and (now - self.last_human_input_time) > self.auto_resume_delay
            #     ):
            #     print("[DEBUG][MODE] auto-resume to AUTO")
            #     self.mode = DriveMode.AUTO
            #    #  return self.agent.run_step()

            # return self.wheel.get_human_control()

        if not self.auto_ready:
            if not self.agent._local_planner:
                return carla.VehicleControl(brake=1.0)
            else:
                print("[DEBUG][AUTO] agent ready")
                self.auto_ready = True
        
        try:
            control = self.agent.run_step()
        except AttributeError as e:
            return carla.VehicleControl(brake=1.0)
                    
        # if self.wheel and self.wheel.detect_intervention():
        #     self.mode = DriveMode.MANUAL
        #     self.last_human_input_time = now
        #     return self.wheel.get_human_control()

        return control