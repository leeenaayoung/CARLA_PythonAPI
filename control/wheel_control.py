import carla
import pygame
import math
from configparser import ConfigParser


class WheelController:
    def __init__(self, config_path="wheel_config.ini"):
        parser = ConfigParser()

        self.control = carla.VehicleControl()

        # --- joystick init ---
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No steering wheel detected")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # --- load config ---
        parser = ConfigParser()
        parser.read(config_path)

        section = parser.sections()[0]  # 첫 섹션 사용
        self.steer_idx = parser.getint(section, "steering_wheel")
        self.throttle_idx = parser.getint(section, "throttle")
        self.brake_idx = parser.getint(section, "brake")

        self.autopilot_button = parser.getint(section, "autopilot_button", fallback=None)

        # 상태 저장
        self._last_steer = 0.0

    def detect_intervention(self,
                            steer_threshold=0.0001,
                            torque_deadzone=0.01):
        pygame.event.pump()
        steer = self.joystick.get_axis(self.steer_idx)
        return abs(steer) > steer_threshold

    def get_human_control(self):
        pygame.event.pump()

        steer_raw = self.joystick.get_axis(self.steer_idx)

        STEER_GAIN = 8.0
        steer_cmd = STEER_GAIN * math.tan(1.1 * steer_raw)
        steer_cmd = max(-1.0, min(1.0, steer_cmd))

        control = carla.VehicleControl()
        control.steer = steer_cmd
        control.throttle = 0.0
        control.brake = 0.0

        return control