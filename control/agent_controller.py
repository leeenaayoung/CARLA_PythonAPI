import carla
import time
from enum import Enum
from agents.navigation.controller import VehiclePIDController

class DriveMode(Enum):
    AUTO = 0
    MANUAL = 1


class AgentController:
    def __init__(self, agent, wheel):
        self.agent = agent
        self.vehicle = agent._vehicle
        self.wheel = wheel

        self.mode = DriveMode.AUTO

        # planner reset control
        self._planner_reset_done = True

        # steer smoothing
        self._last_steer = 0.0
        self.MAX_STEER_RATE = 0.1

        # handover smoothing
        self._handover_alpha = None  
        self.HANDOVER_STEPS = 10
        self._post_reset_hold = 0
        self._force_planner_reset = False

        # global destination
        self._global_destination = None

        # simulator state debug
        self._junction_seen = False
        self._slowdown_seen = False
        self._traffic_light_affected = False
        self._hard_brake_seen = False
        self._arrived_logged = False

    # --------------------------------------------------------------
    # utility
    # --------------------------------------------------------------
    def _reset_agent_planner(self):
        lp = self.agent._local_planner

        if hasattr(lp, "_waypoints_queue"):
            lp._waypoints_queue.clear()

        lp.target_waypoint = None
        lp.target_road_option = None

        ego_loc = self.vehicle.get_location()

        lp._vehicle_controller = VehiclePIDController(
            lp._vehicle,
            args_lateral=lp._args_lateral_dict,
            args_longitudinal=lp._args_longitudinal_dict,
            offset=lp._offset,
            max_throttle=lp._max_throt,
            max_brake=lp._max_brake,
            max_steering=lp._max_steer
        )

        self.agent.set_destination(
            self._global_destination,
            start_location=ego_loc,
            clean_queue=True
        )
    
    # manual_control speed control
    def _get_speed_kmh(self):
        v = self.vehicle.get_velocity()
        return 3.6 * (v.x**2 + v.y**2 + v.z**2) ** 0.5

    # --------------------------------------------------------------
    # main step
    # --------------------------------------------------------------
    def step(self, interrupt=None):

        # ----------------------------------------------------------
        # mode toggle (P key)
        # ----------------------------------------------------------
        if interrupt and interrupt.toggle_mode_requested:
            interrupt.toggle_mode_requested = False

            if self.mode == DriveMode.AUTO:
                self.mode = DriveMode.MANUAL
                print("[MODE] AUTO -> MANUAL")

            else:
                self.mode = DriveMode.AUTO
                
                self._force_planner_reset = True
                self._post_reset_hold = 20

                self._handover_alpha = 0.0
                self._handover_step = 1.0 / self.HANDOVER_STEPS
                print("[MODE] MANUAL -> AUTO")

        # planner reset (ONLY ONCE, before run_step)
        if self.mode == DriveMode.AUTO and self._force_planner_reset:
            self._reset_agent_planner()
            self._force_planner_reset = False
            self._handover_alpha = None

        # drive mode
        if self.mode == DriveMode.AUTO:
            try:
                control = self.agent.run_step()
            except Exception as e:
                print("[WARN] run_step failed:", e)
                return carla.VehicleControl(brake=0.3)
        else:
            # MANUAL mode → base control from human
            control = carla.VehicleControl()
            if self.wheel:
                human = self.wheel.get_human_control()

                speed = self._get_speed_kmh()
                max_speed = self.agent._behavior.max_speed  # km/h

                throttle = human.throttle
                brake = human.brake

                # soft speed cap
                if speed > max_speed:
                    throttle = 0.0
                elif speed > max_speed - 5:
                    throttle *= 0.5

                control.steer = human.steer
                control.throttle = throttle
                control.brake = brake

        if self.agent.done() and not self._arrived_logged:
            print("[DEBUG] Destination reached")
            self._arrived_logged = True

        # waypoint debug
        # wp = getattr(self.agent, "_incoming_waypoint", None)
        # if wp is not None and wp.is_junction:
        #     self._junction_seen = True

        # if hasattr(self.agent, "_vehicle_state"):
        #     if self.agent._vehicle_state:
        #         self._collision_seen = True

        # if control.brake > 0.8:
        #     self._hard_brake_seen = True
        
        # handover smoothing (MANUAL → AUTO)
        if (
                self._handover_alpha is not None
                and self.wheel
                and self.mode == DriveMode.AUTO
            ):

            human = self.wheel.get_human_control()
            a = self._handover_alpha

            control.steer = (1 - a) * human.steer + a * control.steer

            self._handover_alpha += self._handover_step
            if self._handover_alpha >= 1.0:
                self._handover_alpha = None

        # low-speed assist
        # vel = self.vehicle.get_velocity()
        # speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5

        # 감속 조건 기록
        # if speed < 1.0:
        #     self._slowdown_seen = True

        return control
