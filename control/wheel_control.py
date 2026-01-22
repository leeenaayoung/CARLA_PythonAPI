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

    def pedal_normalize(self, raw, deadzone=0.05):
        val = (1.0 - raw) / 2.0

        # deadzone
        if val < deadzone:
            val = 0.0

        # re-normalize (optional but recommended)
        return min(1.0, max(0.0, val))

    def detect_intervention(self,
                            steer_th=0.0001,
                            pedal_th=0.02):
        pygame.event.pump()
        steer = self.joystick.get_axis(self.steer_idx)
        throttle_raw = self.joystick.get_axis(self.throttle_idx)
        brake_raw = self.joystick.get_axis(self.brake_idx)

        # pedal normalize
        # throttle = (1.0 - throttle_raw) / 2.0
        # brake = (1.0 - brake_raw) / 2.0
        throttle = self.pedal_normalize(throttle_raw, deadzone=0.02)
        brake = self.pedal_normalize(brake_raw, deadzone=0.08)


        return (
            abs(steer) > steer_th or
            abs(throttle) > pedal_th or
            abs(brake) > pedal_th
        )

    def get_human_control(self, clock=None):
        pygame.event.pump()

        steer_raw = self.joystick.get_axis(self.steer_idx)
        throttle_raw = self.joystick.get_axis(self.throttle_idx)
        brake_raw = self.joystick.get_axis(self.brake_idx)

        # debug
        # print(f"[DEBUG] throttle_raw={throttle_raw:.3f}, brake_raw={brake_raw:.3f}")

        # steering
        STEER_GAIN = 8.0
        steer_cmd = STEER_GAIN * math.tan(1.1 * steer_raw)
        steer_cmd = max(-1.0, min(1.0, steer_cmd))

        # pedals
        throttle = self.pedal_normalize(throttle_raw, deadzone=0.02)
        brake = self.pedal_normalize(brake_raw, deadzone=0.08)


        control = carla.VehicleControl()
        control.steer = steer_cmd
        control.throttle = throttle
        control.brake = brake

        return control