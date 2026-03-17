"""
ForceSeatMI_Carla.py

CARLA -> Python client -> ForceSeatMI (Telemetry ACE) bridge example.

Checklist: collect these inputs from CARLA each tick
1) Timing
- Snapshot timestamp and delta_seconds
- Fallback monotonic delta when delta_seconds is zero

2) Vehicle kinematics (minimum set)
- vehicle.get_transform()
- vehicle.get_velocity()
- vehicle.get_acceleration() (optional, currently derived by velocity diff)
- vehicle.get_control() (throttle, brake, steer, hand_brake, reverse, gear)

3) Strongly recommended for quality
- wheel contact/suspension data (if available in your CARLA version)
- drivetrain data (engine RPM/current gear from physics extension)
- collision impulse / lane invasion / curb events

4) Mapping into FSMI_TelemetryACE
- vehicleForwardSpeed
- bodyLinearVelocity.{forward,right,upward}
- bodyLinearAcceleration.{forward,right,upward}
- bodyAngularVelocity.{pitch,roll,yaw}
- bodyPitch, bodyRoll
- gearNumber, accelerationPedalPosition, brakePedalPosition, clutchPedalPosition
- rpm, maxRpm
- sideSlip, roadHarshness, g
- extraTranslation / extraRotation / userAux (optional custom effects)
"""

import argparse
import math
import time
from dataclasses import dataclass

import carla

from ForceSeatMI import ForceSeatMI
from ForceSeatMI_Structs import FSMI_TelemetryACE
from ctypes import sizeof


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


class ForceSeatCarlaBridge:
    def __init__(self, profile_name: str, low_pass_alpha: float) -> None:
        self.fsmi = ForceSeatMI()
        self.profile_name = profile_name

    def start(self) -> None:
        if not self.fsmi.activate_profile(self.profile_name):
            raise RuntimeError(f"Failed to activate ForceSeat profile: {self.profile_name}")
        if not self.fsmi.begin_motion_control():
            raise RuntimeError("Failed to start ForceSeat motion control")

    def stop(self) -> None:
        try:
            self.fsmi.end_motion_control()
        finally:
            self.fsmi.delete()

    def build_telemetry(self, vehicle: carla.Vehicle, dt: float) -> FSMI_TelemetryACE:
        transform = vehicle.get_transform()
        # velocity = vehicle.get_velocity()
        control = vehicle.get_control()

        forward = transform.get_forward_vector()
        right = transform.get_right_vector()
        up = transform.get_up_vector()

        acc_w = vehicle.get_acceleration()
        anglular_velocity = vehicle.get_angular_velocity()
        vel = vehicle.get_velocity()

        # World velocity projected onto vehicle local axes [m/s]
        # forward_speed = velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z
        # right_speed = velocity.x * right.x + velocity.y * right.y + velocity.z * right.z
        # up_speed = velocity.x * up.x + velocity.y * up.y + velocity.z * up.z

        w_world = carla.Vector3D(
            math.radians(anglular_velocity.x),
            math.radians(anglular_velocity.y),
            math.radians(anglular_velocity.z),
        )

        # World-to-Body Transition
        omega_f = w_world.x * forward.x + w_world.y * forward.y + w_world.z * forward.z
        omega_r = w_world.x * right.x   + w_world.y * right.y   + w_world.z * right.z
        omega_u = w_world.x * up.x      + w_world.y * up.y      + w_world.z * up.z

        acc_f = acc_w.x * forward.x + acc_w.y * forward.y + acc_w.z * forward.z
        acc_r = acc_w.x * right.x   + acc_w.y * right.y   + acc_w.z * right.z
        acc_u = acc_w.x * up.x      + acc_w.y * up.y      + acc_w.z * up.z

        forward_speed_mps = vel.x * forward.x + vel.y * forward.y + vel.z * forward.z

        telemetry = FSMI_TelemetryACE()
        telemetry.structSize = sizeof(FSMI_TelemetryACE)

        telemetry.accelerationPedalPosition = int(clamp(control.throttle * 100.0, 0.0, 100.0))
        telemetry.brakePedalPosition = int(clamp(control.brake * 100.0, 0.0, 100.0))
        telemetry.clutchPedalPosition = 0

        gear = int(control.gear)
        if control.reverse and gear >= 0:
            gear = -1
        telemetry.gearNumber = gear

        # CARLA Python API usually exposes max RPM via physics control, current RPM may be unavailable.
        telemetry.rpm = 0
        try:
            physics = vehicle.get_physics_control()
            telemetry.maxRpm = int(max(0.0, float(getattr(physics, "max_rpm", 0.0))))
        except RuntimeError:
            telemetry.maxRpm = 0

        telemetry.vehicleForwardSpeed = forward_speed_mps

        telemetry.bodyLinearAcceleration.forward = -float(acc_f)
        telemetry.bodyLinearAcceleration.right = float(acc_r)
        telemetry.bodyLinearAcceleration.upward = float(acc_u)
        telemetry.bodyAngularVelocity.pitch = float(omega_r)
        telemetry.bodyAngularVelocity.roll = float(omega_f)
        # telemetry.bodyAngularVelocity.yaw = - anglular_velocity.z
        telemetry.bodyAngularVelocity.yaw = -float(omega_u)


        # # Optional helper channels
        # telemetry.sideSlip = float(math.atan2(right_speed, max(0.5, abs(forward_speed))))
        # telemetry.roadHarshness = float(clamp(abs(telemetry.bodyLinearAcceleration.upward) / 5.0, 0.0, 1.0))
        # telemetry.g = float(telemetry.bodyLinearAcceleration.upward / 9.80665)

        # telemetry.extraRotation.pitch = 0.0
        # telemetry.extraRotation.roll = 0.0
        # telemetry.extraRotation.yaw = 0.0
        # telemetry.extraTranslation.forward = 0.0
        # telemetry.extraTranslation.right = 0.0
        # telemetry.extraTranslation.upward = 0.0

        # for i in range(8):
        #     telemetry.userAux[i] = 0.0

        return telemetry


def find_vehicle(world: carla.World, role_name: str, timeout_sec: float) -> carla.Vehicle:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name", "") == role_name:
                return actor
        time.sleep(0.2)
    raise RuntimeError(f"Vehicle with role_name='{role_name}' not found within {timeout_sec:.1f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description="CARLA -> ForceSeatMI Telemetry ACE bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role-name", default="hero")
    parser.add_argument("--profile", default="SDK - Vehicle Telemetry ACE")
    parser.add_argument("--vehicle-timeout", type=float, default=20.0)
    parser.add_argument("--low-pass-alpha", type=float, default=0.7)
    parser.add_argument("--print-every", type=float, default=1.0, help="seconds")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)

    world = client.get_world()
    vehicle = find_vehicle(world, args.role_name, args.vehicle_timeout)

    bridge = ForceSeatCarlaBridge(profile_name=args.profile, low_pass_alpha=clamp(args.low_pass_alpha, 0.0, 1.0))
    bridge.start()

    last_wall = time.perf_counter()
    last_print = last_wall

    try:
        while True:
            snapshot = world.wait_for_tick()
            dt = float(snapshot.timestamp.delta_seconds)

            if dt <= 1e-5:
                now = time.perf_counter()
                dt = max(1e-3, now - last_wall)
                last_wall = now
            else:
                last_wall = time.perf_counter()

            telemetry = bridge.build_telemetry(vehicle, dt)
            ok = bridge.fsmi.send_telemetry_ace(telemetry)
            if not ok:
                print("[WARN] send_telemetry_ace returned False")

            now = time.perf_counter()
            if now - last_print >= args.print_every:
                speed_kph = telemetry.vehicleForwardSpeed * 3.6
                print(
                    f"speed={speed_kph:6.1f} km/h "
                    f"gear={telemetry.gearNumber:2d} "
                    f"acc_f={telemetry.bodyLinearAcceleration.forward:7.3f} "
                    f"yaw_rate={telemetry.bodyAngularVelocity.yaw:7.3f}"
                )
                last_print = now

    except KeyboardInterrupt:
        print("Stopping bridge...")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
