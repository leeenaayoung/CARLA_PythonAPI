import carla

class ControlManager:
    def __init__(self, vehicle, keyboard, wheel=None):
        self.vehicle = vehicle
        self.keyboard = keyboard
        self.wheel = wheel

        self.autopilot = False
        self._last_autopilot = False

    def tick(self, clock):
        wheel_request = False
        if self.wheel and hasattr(self.wheel, "autopilot_requested"):
            wheel_request = self.wheel.autopilot_requested

        self.autopilot = self.keyboard.autopilot_enabled or wheel_request

        if self.autopilot != self._last_autopilot:
            self.vehicle.set_autopilot(self.autopilot)
            self._last_autopilot = self.autopilot

        if self.autopilot:
            return

        # if self.wheel and self.wheel.has_input():
        #     print("CONTROL SOURCE: WHEEL")
        #     control = self.wheel.get_control(clock)
        # else:
        #     print("CONTROL SOURCE: KEYBOARD")
        #     control = self.keyboard.get_control(clock)

        # self.vehicle.apply_control(control)

        kb_control = self.keyboard.get_control(clock)

        wheel_control = None
        if self.wheel:
            wheel_control = self.wheel.get_control(clock)

        # control 병합
        control = carla.VehicleControl()

        # control 방식 혼합 -> 리팩토링 예정
        if wheel_control:
            control.steer = wheel_control.steer
        else:
            control.steer = kb_control.steer

        control.throttle = kb_control.throttle
        control.brake = kb_control.brake
        control.hand_brake = kb_control.hand_brake

        self.vehicle.apply_control(control)