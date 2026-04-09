import math

import carla
import pygame
from configparser import ConfigParser


class WheelController:
    def __init__(self, config_path="wheel_config.ini"):
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
        sections = parser.sections()
        if not sections:
            raise RuntimeError("No section found in wheel_config.ini")
        section = sections[0]

        self.steer_idx = parser.getint(section, "steering_wheel")
        self.throttle_idx = parser.getint(section, "throttle")
        self.brake_idx = parser.getint(section, "brake")
        self.handbrake_idx = parser.getint(section, "handbrake", fallback=None)
        self.gear_forward_idx = parser.getint(section, "gear_forward", fallback=None)
        self.gear_reverse_idx = parser.getint(section, "gear_reverse", fallback=None)

        self.autopilot_button = parser.getint(section, "autopilot_button", fallback=None)

        # button edge state cache
        self._button_states = {}

        self._is_reverse = False

        # keep compatibility placeholder
        self._last_steer = 0.0

    def pedal_normalize(self, raw, deadzone=0.05):
        val = (1.0 - raw) / 2.0

        # deadzone
        if val < deadzone:
            val = 0.0

        # re-normalize (optional but recommended)
        return min(1.0, max(0.0, val))

    def _was_button_pressed(self, button_idx):
        previous = self._button_states.get(button_idx, False)
        current = bool(self.joystick.get_button(button_idx))
        self._button_states[button_idx] = current
        return current and not previous

    def _update_drive_direction_by_buttons(self):
        if self.gear_forward_idx is not None and self._was_button_pressed(self.gear_forward_idx):
            self._is_reverse = False
        if self.gear_reverse_idx is not None and self._was_button_pressed(self.gear_reverse_idx):
            self._is_reverse = True

    def detect_intervention(self, steer_th=0.0001, pedal_th=0.02):
        pygame.event.pump()
        steer = self.joystick.get_axis(self.steer_idx)
        throttle_raw = self.joystick.get_axis(self.throttle_idx)
        brake_raw = self.joystick.get_axis(self.brake_idx)

        throttle = self.pedal_normalize(throttle_raw, deadzone=0.02)
        brake = self.pedal_normalize(brake_raw, deadzone=0.08)

        direction_input = False
        if self.gear_reverse_idx is not None:
            direction_input |= self._was_button_pressed(self.gear_reverse_idx)
        if self.gear_forward_idx is not None:
            direction_input |= self._was_button_pressed(self.gear_forward_idx)

        return (
            abs(steer) > steer_th
            or abs(throttle) > pedal_th
            or abs(brake) > pedal_th
            or direction_input
        )

    def get_human_control(self, clock=None):
        pygame.event.pump()

        steer_raw = self.joystick.get_axis(self.steer_idx)
        throttle_raw = self.joystick.get_axis(self.throttle_idx)
        brake_raw = self.joystick.get_axis(self.brake_idx)

        STEER_GAIN = 8.0
        steer_cmd = STEER_GAIN * math.tan(1.1 * steer_raw)
        steer_cmd = max(-1.0, min(1.0, steer_cmd))

        throttle = self.pedal_normalize(throttle_raw, deadzone=0.02)
        brake = self.pedal_normalize(brake_raw, deadzone=0.08)

        self._update_drive_direction_by_buttons()

        control = carla.VehicleControl()
        control.steer = steer_cmd
        control.throttle = throttle
        control.brake = brake
        control.manual_gear_shift = False
        control.gear = 0
        control.reverse = self._is_reverse
        control.hand_brake = bool(self.joystick.get_button(self.handbrake_idx)) if self.handbrake_idx is not None else False

        return control
