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
        self.destination = None
        self._last_steer = 0.0
        self.max_steer_rate = 0.05

        self.auto_enter_time = None
        self._transition_stop = False

    def _limit_steer_rate(self, steer):
        delta = steer - self._last_steer
        if abs(delta) > self.max_steer_rate:
            steer = self._last_steer + self.max_steer_rate * (1 if delta > 0 else -1)
        self._last_steer = steer
        return steer

    def step(self, interrupt=None):
        now = time.time()

        # Check for human intervention
        if interrupt and interrupt.toggle_mode_requested:
            if self.mode == DriveMode.AUTO:
                self.mode = DriveMode.MANUAL
                self.manual_by_toggle = True
                self._transition_stop = True
                print("[DEBUG][MODE] toggled -> DriveMode.MANUAL")
            else:
                self.mode = DriveMode.AUTO
                # self.auto_ready = False
                self.manual_by_toggle = False
                self._last_steer = 0.0
                if self.auto_enter_time is None:         
                    self.auto_enter_time = time.time()

                try:
                    ego = self.agent._vehicle
                    dest = self.agent._destination

                    self.agent.set_destination(dest)
                    print("[DEBUG][MODE] -> AUTO (replan from current pose)")
                except Exception as e:
                    print("[WARN][AUTO] replan failed:", e)
    
                print("[DEBUG][MODE] toggled -> DriveMode.AUTO")

            self.last_human_input_time = now
            interrupt.toggle_mode_requested = False
        
        if self.mode == DriveMode.MANUAL:
            if self.wheel:
                if self._transition_stop:
                    self._transition_stop = False
                    return carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                return self.wheel.get_human_control()
            else:
                return carla.VehicleControl()
        
        try:
            control = self.agent.run_step()

            AUTO_STABILIZE_TIME = 0.5

            # elapsed = time.time() - self.auto_enter_time
            if self.auto_enter_time is None:
                elapsed = float("inf")
            else:
                elapsed = time.time() - self.auto_enter_time


            if elapsed < AUTO_STABILIZE_TIME:
                # 속도는 억제
                control.throttle = 0.0
                control.brake = 1.0

                # 조향만 천천히 정렬
                control.steer = self._limit_steer_rate(control.steer)
            else:
                control.brake = 0.0
                self.auto_enter_time = None

            return control

            # control.steer = self._limit_steer_rate(control.steer)
        except AttributeError as e:
            print("[ERROR][AUTO]", e)
            return carla.VehicleControl(brake=1.0)

        return control
