import carla
import math
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

# client = carla.Client('localhost', 2000)
# client.set_timeout(200.0) 
# client.start_recorder('recording.log')

#  env setup
def setup_world(client, town="Town04_Opt") -> carla.World:
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
                                        carla.Location(x=0.07, y=-0.5, z=1.18),
                                        carla.Rotation(pitch=-9.0, yaw=0.0, roll=0.0)
                                        ),
                                        attach_to=vehicle
                                )

    return camera

# ego vehicle spawn
def spawn_vehicle(world):
    blueprints = world.get_blueprint_library()
    vehicle_bp = blueprints.filter('vehicle.ford.mustang')[0]
    vehicle_bp.set_attribute("role_name", "hero")

    # same location in Unreal Engine, but *0.01 scale
    start_raw = carla.Transform(
                                carla.Location(x=258.46, y=-180.46, z=0.40),  # Town04_Opt start = (x=11.77, y=-43.63, z=0.28), VehicleSpawnPoint294
                                carla.Rotation(pitch=0.0, yaw=-89.82, roll=0.0)
                                )

    spawn_wp = world.get_map().get_waypoint(
        start_raw.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    if spawn_wp is None:
        raise RuntimeError(f"No driving waypoint near start_raw: {start_raw}")

    spawn_tf = spawn_wp.transform
    spawn_tf.location.z += 0.3

    print("start_raw:", start_raw)
    print("projected spawn_tf:", spawn_tf)

    vehicle = world.try_spawn_actor(vehicle_bp, spawn_tf)
    if vehicle is None:
        raise RuntimeError(f"Failed to spawn ego at {spawn_tf}")

    world.tick()
    # print("vehicle_tf after tick:", vehicle.get_transform())
    return vehicle

def spawn_npc_in_front(world, ego_vehicle, tm, distance=20.0, z_offset=0.3):
    bp_lib = world.get_blueprint_library()

    # ego랑 같은 차종만 피하고 싶으면 적당히 필터
    candidates = bp_lib.filter('vehicle.*')
    npc_bp = None

    for bp in candidates:
        if bp.id != 'vehicle.tesla.model3':
            npc_bp = bp
            break

    if npc_bp is None:
        npc_bp = candidates[0]

    ego_wp = world.get_map().get_waypoint(
        ego_vehicle.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    if ego_wp is None:
        print("No ego waypoint found")
        return None

    # 여러 거리로 재시도
    for d in [distance, distance + 8.0, distance + 15.0, distance + 25.0]:
        next_wps = ego_wp.next(d)
        if not next_wps:
            continue

        target_wp = next_wps[0]
        spawn_tf = target_wp.transform
        spawn_tf.location.z += z_offset

        npc = world.try_spawn_actor(npc_bp, spawn_tf)
        if npc is not None:
            npc.set_autopilot(True, tm.get_port())

            # ego 바로 앞차처럼 너무 느리거나 멈추지 않게 약간 설정 가능
            tm.vehicle_percentage_speed_difference(npc, -10.0)  # 살짝 빠르게
            tm.distance_to_leading_vehicle(npc, 4.0)

            print("Front NPC spawned at:", spawn_tf)
            return npc

    print("Failed to spawn front NPC")
    return None

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
    pygame.mixer.init()

    os.environ['SDL_VIDEO_WINDOW_POS'] = '0,30'

    # screen = pygame.display.set_mode((1080, 600))
    info = pygame.display.Info()
    SCREEN_W, SCREEN_H = info.current_w, info.current_h - 30
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))

    screen.fill((30, 30, 30))
    pygame.display.flip()

    # background music setup
    print("mixer init ok:", pygame.mixer.get_init())

    wind_path = os.path.join("assets", "audio", "wind_sample.wav")
    print("cwd:", os.getcwd())
    print("wind exists:", os.path.exists(wind_path), wind_path)

    pygame.mixer.music.load(wind_path)
    pygame.mixer.music.set_volume(1.0)
    pygame.mixer.music.play(loops=-1)

    print("music busy right after play:", pygame.mixer.music.get_busy())

    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)    # CARLA server connect
    client.set_timeout(20.0)

    world = setup_world(client)  

    mode_toggle  = KeyboardModeToggle()

    # actors = world.get_actors()

    # for actor in actors:
    #     print(actor.id, actor.type_id, actor.get_transform())

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
    agent = BehaviorAgent(ego, behavior='drive_mode')

    # NPC vehicles
    tm = setup_traffic_manager(client, sync=True)
    # tm.set_random_device_seed(0)
    # npc_vehicles = spawn_npc_vehicles(world, tm, num_vehicles=30)
    tm.set_random_device_seed(0)

    front_npc = spawn_npc_in_front(world, ego, tm, distance=20.0)
    npc_vehicles = []

    if front_npc is not None:
        npc_vehicles.append(front_npc)

    # 나머지 차량들 랜덤 스폰
    other_npcs = spawn_npc_vehicles(world, tm, num_vehicles=29)
    npc_vehicles.extend(other_npcs)

    spawn_points = world.get_map().get_spawn_points()

    ego_wp = world.get_map().get_waypoint(
        ego.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    # fixed target location
    # target_wp = world.get_map().get_waypoint(
    #     spawn_points[60].location,
    #     project_to_road=True,
    #     lane_type=carla.LaneType.Driving
    # )
    
    target_raw = carla.Transform(
                                carla.Location(x=246.87, y=-172.75, z=0.60),  # Town04_Opt target = (x=314.88, y=-213.09, z=0.28), VehicleSpawnPoint296
                                carla.Rotation(pitch=0.0, yaw=-179.67, roll=0.0)
                                )
    
    target_wp = world.get_map().get_waypoint(
                    target_raw.location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving
                )
    
    if target_wp is None:
        raise RuntimeError(f"No driving waypoint near target_raw: {target_raw}")

    target_location = target_wp.transform.location
    
    start_loc = ego.get_location()
    end_loc = target_location
    # agent.set_destination(target_location)

    agent.set_destination(end_loc, start_location=start_loc)

    start_wp = world.get_map().get_waypoint(
    ego.get_location(),
    project_to_road=True,
    lane_type=carla.LaneType.Driving
    )

    end_wp = world.get_map().get_waypoint(
        target_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    print("start road/lane:", start_wp.road_id, start_wp.lane_id)
    print("end road/lane:", end_wp.road_id, end_wp.lane_id)

    route = agent.trace_route(start_wp, end_wp)
    print("route length:", len(route))

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
    camera = setup_camera(world, ego, width=SCREEN_W, height=SCREEN_H)

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