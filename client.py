import carla
import csv
import os
import pygame
import numpy as np
from datetime import datetime

from npc.generate_npc import spawn_npc_vehicles

from control.agent_controller import AgentController
from control.wheel_control import WheelController
from control.KeyboardModeToggle import KeyboardModeToggle

from agents.navigation.behavior_agent import BehaviorAgent

client = carla.Client('localhost', 2000)
client.set_timeout(2000.0) 
# client.start_recorder('recording.log')

#  env setup
def setup_world(client, town="Town10HD_Opt") -> carla.World:
    client.load_world(town)
    world = client.get_world()
    map = world.get_map()

    # set weather
    # weather = carla.WeatherParameters(
    #             cloudiness=30.0,
    #             precipitation=60.0,
    #             precipitation_deposits=60.0,
    #             wind_intensity=30.0,
    #             sun_altitude_angle=45.0,
    #             sun_azimuth_angle=300.0,
    #             fog_density=20.0,
    #             fog_distance=0.8,
    #             fog_falloff=0.8,
    #             wetness=40.0,
    #             scattering_intensity=30.0,
    #             mie_scattering_scale=0.03,
    #             rayleigh_scattering_scale=0.0,
    #             dust_storm=50.0
    #         )
    
    # world.set_weather(weather)

    # weather = world.get_weather()

    # for _ in range(20):
    #     weather.fog_density += 5.0
    #     weather.precipitation += 15.0
    #     weather.cloudiness += 10.0
    #     weather.dust_storm += 10.0
    #     world.set_weather(weather)

    settings = world.get_settings()
    settings.synchronous_mode = True    # synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    actors = world.get_actors()
    for actor in actors:
        if actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker"):
            actor.destroy()

    world.tick()

    print("Connected to:", world.get_map().name)
    return world

# TM setup for npc
def setup_traffic_manager(client, sync=True, seed=None):
    tm = client.get_trafficmanager(8000)

    if sync:
        tm.set_synchronous_mode(True)

    if seed is not None:
        tm.set_random_device_seed(seed)

    tm.set_global_distance_to_leading_vehicle(4.0)
    tm.global_percentage_speed_difference(0.0)

    return tm

# camera setup
def setup_camera(world, vehicle, width=800, height=600):
    camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(width))
    camera_bp.set_attribute("image_size_y", str(height))
    camera_bp.set_attribute("fov", "65")
    # camera_bp.set_attribute("enable_postprocess_effects", "false")
    camera_bp.set_attribute("exposure_mode", "manual")

    camera = world.spawn_actor(
                                camera_bp,
                                carla.Transform(
                                        carla.Location(x=0.05, y=-0.23, z=1.18),
                                        carla.Rotation(pitch=-9.0, yaw=0.0, roll=0.0)
                                        ),
                                        attach_to=vehicle
                                    )

    return camera

# ego vehicle spawn
def spawn_vehicle(world):
    blueprints = world.get_blueprint_library()
    # vehicle_bp = blueprints.filter('vehicle')[0]
    vehicle_bp = blueprints.filter('vehicle.tesla.model3')[0]
    vehicle_bp.set_attribute("role_name", "hero")

    spawn_points = world.get_map().get_spawn_points()
    # debug
    print("Spawn points:", len(spawn_points))

    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])    # fixed spawn point
    # debug
    if vehicle is None:
        print("Spawn failed")
    else:
        print("Spawned vehicle:", vehicle.id)

    return vehicle

#spectator setup
def setup_spectator(world, vehicle):
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(x=0,z=1.3),
            carla.Rotation(pitch=-30)
        )
    )

# spectator follow
def follow_ego_spectator(world, ego, height=2.5, pitch=-20):
    spectator = world.get_spectator()
    transform = ego.get_transform()

    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(z=height),
            carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw)
        )
    )

def main():
    pygame.init()
    screen = pygame.display.set_mode((1080, 600))
    screen.fill((30, 30, 30))
    pygame.display.flip()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)    # CARLA server connect
    client.set_timeout(20.0)

    world = setup_world(client)  

    mode_toggle  = KeyboardModeToggle()

    collision_sensor = None
    camera = None
    log_file = None

    # ego vehicle
    ego = spawn_vehicle(world)
    world.tick() 

    setup_spectator(world, ego)

    ego.apply_control(carla.VehicleControl(brake=1.0))
    for _ in range(10):
        world.tick()

    # add coliison sensor
    collision_bp = world.get_blueprint_library().find("sensor.other.collision")

    collision_flag = {"value": 0}
    collision_intensity = {"value": 0.0}

    def collision_callback(event):
        impulse = event.normal_impulse
        intensity = (impulse.x**2 + impulse.y**2 + impulse.z**2) ** 0.5

        collision_flag["value"] = 1
        collision_intensity["value"] = intensity

    collision_sensor = world.spawn_actor(
        collision_bp,
        carla.Transform(),
        attach_to=ego
    )

    collision_sensor.listen(collision_callback)

    # agent
    agent = BehaviorAgent(ego, behavior='custom')

    # NPC vehicles
    tm = setup_traffic_manager(client, sync=True)
    tm.set_random_device_seed(0)
    npc_vehicles = spawn_npc_vehicles(world, tm, num_vehicles=15) 

    spawn_points = world.get_map().get_spawn_points()

    ego_wp = world.get_map().get_waypoint(
        ego.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    # fixed target location
    target_wp = world.get_map().get_waypoint(
        spawn_points[5].location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    target_location = target_wp.transform.location
    
    agent.set_destination(target_location)

    for _ in range(10):
        world.tick()

    # steering_wheel setup
    wheel = None
    try:
        wheel = WheelController(config_path="wheel_config.ini")
        print("Wheel controller enabled")
    except Exception as e:
        print("Wheel controller not available:", e)

    # agent controller
    agent_controller = AgentController(agent, wheel)
    agent_controller._global_destination = target_location

    # setup camera
    camera = setup_camera(world, ego, width=1080, height=600)

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        rgb = array[:, :, :3][:, :, ::-1]  # BGRA → RGB

        surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        screen.blit(surface, (0, 0))
        pygame.display.flip()
    
    for _ in range(5):
        world.tick()

    camera.listen(camera_callback)

    print("Simulation running... Ctrl+C to quit")

    # vehicle state logging setup
    log_dir = "ego_state"
    os.makedirs(log_dir, exist_ok=True)

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    log_path = os.path.join(log_dir, f"ego_log_{timestamp_str}.csv")

    log_file = open(log_path, "w", newline="")
    
    writer = csv.writer(log_file)
    writer.writerow([
        "time",
        "x","y","z",
        "yaw","pitch","roll",
        "vx","vy","vz",
        "ax","ay","az",
        "wx","wy","wz",
        "speed_kmh",
        "throttle","steer","brake","gear",
        "lane_id","road_id","is_junction",
        "collision","collision_intensity"
    ])

    try:
        while True:
            # world.wait_for_tick() # async mode
            world.tick()  # synchronous mode
            clock.tick(60)
            pygame.event.pump()

            mode_toggle.parse_events()

            control = agent_controller.step(mode_toggle)
            ego.apply_control(control)

            state = agent_controller.get_vehicle_state()
            timestamp = world.get_snapshot().timestamp.elapsed_seconds

            writer.writerow([
                timestamp,
                state["x"], state["y"], state["z"],
                state["yaw"], state["pitch"], state["roll"],
                state["vx"], state["vy"], state["vz"],
                state["ax"], state["ay"], state["az"],
                state["wx"], state["wy"], state["wz"],
                state["speed_kmh"],
                state["throttle"], state["steer"], state["brake"], state["gear"],
                state["lane_id"], state["road_id"], state["is_junction"],
                collision_flag["value"],
                collision_intensity["value"]
            ])

            collision_flag["value"] = 0
            collision_intensity["value"] = 0.0

            follow_ego_spectator(world, ego)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Cleaning up actors...")
        collision_sensor.stop()
        collision_sensor.destroy()
        log_file.close()
        ego.destroy()
        camera.stop()
        camera.destroy()
        pygame.quit()

if __name__ == "__main__":
    main()