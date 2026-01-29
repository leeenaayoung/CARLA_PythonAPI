import carla
import time
from enum import Enum


class DriveMode(Enum):
    AUTO = 0
    MANUAL = 1


class AgentController:
    """
    AUTO   : BehaviorAgent full control
    MANUAL : BehaviorAgent runs in background, steer overridden by human
    """

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
        current_wp = self.agent._map.get_waypoint(
            ego_loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if hasattr(self.agent, "_destination") and self.agent._destination:
            target_loc = self.agent._destination
        else:
            next_wps = current_wp.next(20.0)
            if not next_wps:
                return
            target_loc = next_wps[0].transform.location

        self.agent.set_destination(
            target_loc,
            start_location=current_wp.transform.location,
            clean_queue=True
        )

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
                self._planner_reset_done = False
                print("[MODE] MANUAL -> AUTO")

        # ----------------------------------------------------------
        # ALWAYS run agent
        # ----------------------------------------------------------
        world = self.vehicle.get_world()
        # control = self.agent.run_step()

        try:
            control = self.agent.run_step()
        except AttributeError as e:
            # BehaviorAgent 내부 waypoint 붕괴 보호
            print("[WARN] BehaviorAgent state invalid, fallback to LocalPlanner")

            control = self.agent._local_planner.run_step()

        print(f"[DEBUG] agent steer = {control.steer:.3f}, throttle = {control.throttle:.3f}")
    
        # ----------------------------------------------------------
        # planner reset (MANUAL -> AUTO, only once)
        # ----------------------------------------------------------
        if self.mode == DriveMode.AUTO and not self._planner_reset_done:
            self._reset_agent_planner()
            self._planner_reset_done = True

        # ----------------------------------------------------------
        # low-speed assist (bias, not override)
        # ----------------------------------------------------------
        vel = self.vehicle.get_velocity()
        speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5

        if speed < 1.0:
            control.throttle = max(control.throttle, 0.35)
            control.brake = 0.0

        # ----------------------------------------------------------
        # steer rate limit (always)
        # ----------------------------------------------------------
        # delta = control.steer - self._last_steer
        # if abs(delta) > self.MAX_STEER_RATE:
        #     control.steer = self._last_steer + self.MAX_STEER_RATE * (1 if delta > 0 else -1)
        # self._last_steer = control.steer

        # ----------------------------------------------------------
        # MANUAL steer override
        # ----------------------------------------------------------
        if self.mode == DriveMode.MANUAL and self.wheel:
            human = self.wheel.get_human_control()
            control.steer = human.steer

        return control
