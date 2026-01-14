import carla
import pygame
from pygame.locals import (
    K_w, K_s, K_a, K_d,
    K_q, K_p, K_m,
    K_COMMA, K_PERIOD,
    K_SPACE,
    KMOD_CTRL
)

class KeyboardController:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.control = carla.VehicleControl()

        self.autopilot_enabled = False
        self.constant_velocity_enabled = False

        self.steer_cache = 0.0

    def tick(self, clock):
        self._handle_events()
        self._update_control(clock)
        self._apply_control()

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

            if event.type == pygame.KEYUP:
                self._handle_keyup(event)

    def _handle_keyup(self, event):
        # Q: 후진 기어 토글
        # P: autopilot 토글
        # M: 수동 변속 토글
        # , / . : 기어 업 / 다운
        # CTRL + W : 정속 주행 토글
        if event.key == K_q:
            if self.control.gear >= 0:
                self.control.gear = -1
            else:
                self.control.gear = 1
            self.control.reverse = self.control.gear < 0

        elif event.key == K_p:
            self.autopilot_enabled = not self.autopilot_enabled
            self.vehicle.set_autopilot(self.autopilot_enabled)

        elif event.key == K_m:
            self.control.manual_gear_shift = not self.control.manual_gear_shift
            self.control.gear = self.vehicle.get_control().gear

        elif self.control.manual_gear_shift:
            if event.key == K_COMMA:
                self.control.gear -= 1
            elif event.key == K_PERIOD:
                self.control.gear += 1

        if event.key == K_w and pygame.key.get_mods() & KMOD_CTRL:
            if self.constant_velocity_enabled:
                self.vehicle.disable_constant_velocity()
                self.constant_velocity_enabled = False
            else:
                self.vehicle.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                self.constant_velocity_enabled = True

    def _update_control(self, clock):
        if self.autopilot_enabled or self.constant_velocity_enabled:
            return

        keys = pygame.key.get_pressed()
        milliseconds = clock.get_time()
        keys = pygame.key.get_pressed()


        # Throttle
        if keys[K_w]:
            self.control.throttle = min(self.control.throttle + 0.1, 1.0)
        else:
            self.control.throttle = 0.0

        # Brake
        if keys[K_s]:
            self.control.brake = min(self.control.brake + 0.2, 1.0)
        else:
            self.control.brake = 0.0

        # Steering (중요)
        steer_inc = 5e-4 * milliseconds
        if keys[K_a]:
            self.steer_cache -= steer_inc
        elif keys[K_d]:
            self.steer_cache += steer_inc
        else:
            self.steer_cache = 0.0

        self.steer_cache = max(-0.7, min(0.7, self.steer_cache))
        self.control.steer = self.steer_cache

        # Hand brake
        self.control.hand_brake = keys[K_SPACE]

    def _apply_control(self):
        if not self.autopilot_enabled:
            self.vehicle.apply_control(self.control)





