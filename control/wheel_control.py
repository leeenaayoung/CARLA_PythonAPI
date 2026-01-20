import carla
import pygame
import math
from configparser import ConfigParser


class WheelController:
    def __init__(self, vehicle, config_path="wheel_config.ini"):
        parser = ConfigParser()

        self.vehicle = vehicle
        self.control = carla.VehicleControl()
        self.autopilot_requested = False

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

    def has_input(self, steer_threshold=0.0001):
        pygame.event.pump()
        steer = self.joystick.get_axis(self.steer_idx)
        print("wheel steer raw:", steer)
        return abs(steer) > steer_threshold

    def get_control(self, clock):
        pygame.event.pump()

        steer_raw = self.joystick.get_axis(self.steer_idx)
        
        # test
        STEER_GAIN = 8.0
        steer_cmd = STEER_GAIN * math.tan(1.1 * steer_raw)
        steer_cmd = max(-1.0, min(1.0, steer_cmd))
        self.control.steer = steer_cmd

        self.control.steer = steer_cmd
        self._last_steer = steer_cmd

        # --- throttle / brake (페달 연결 전: 0 고정) ---
        self.control.throttle = 0.0
        self.control.brake = 0.0

        # --- autopilot ---
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if self.autopilot_button is not None and event.button == self.autopilot_button:
                    self.autopilot_requested = not self.autopilot_requested

        return self.control