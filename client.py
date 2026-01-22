import carla
import random
import pygame
import numpy as np

from npc.generate_npc import spawn_npc_vehicles

from control.agent_controller import AgentController
from control.wheel_control import WheelController
from control.KeyboardModeToggle import KeyboardModeToggle

from agents.navigation.basic_agent import BasicAgent

client = carla.Client('localhost', 2000)
client.set_timeout(10.0) 
client.load_world('Town04') 
# client.start_recorder('recording.log')

# 환경 세팅
def setup_world(client):
    world = client.get_world()
    map = world.get_map()

    settings = world.get_settings()
    settings.synchronous_mode = True    # 동기 모드 활성화
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    print("Connected to:", world.get_map().name)
    return world

# TM 설정
def setup_traffic_manager(client, sync=True, seed=None):
    tm = client.get_trafficmanager(8000)

    if sync:
        tm.set_synchronous_mode(True)

    if seed is not None:
        tm.set_random_device_seed(seed)

    tm.set_global_distance_to_leading_vehicle(2.5)
    tm.global_percentage_speed_difference(0.0)

    return tm

# 카메라 세팅
def setup_camera(world, vehicle, width=800, height=600):
    camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(width))
    camera_bp.set_attribute("image_size_y", str(height))
    camera_bp.set_attribute("fov", "65")

    camera = world.spawn_actor(
                                camera_bp,
                                carla.Transform(
                                        carla.Location(x=0.05, y=-0.23, z=1.18),
                                        carla.Rotation(pitch=-9.0, yaw=0.0, roll=0.0)
                                        ),
                                        attach_to=vehicle
                                    )

    return camera

# 차량 스폰
def spawn_vehicle(world):
    blueprints = world.get_blueprint_library()
    # vehicle_bp = blueprints.filter('vehicle')[0]
    vehicle_bp = blueprints.filter('vehicle.tesla.model3')[0]
    spawn_points = world.get_map().get_spawn_points()
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
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

    client = carla.Client('localhost', 2000)    # CARLA 서버 연결
    client.set_timeout(10.0)

    world = setup_world(client)  # 환경 세팅

    mode_toggle  = KeyboardModeToggle()

    # ego vehicle
    ego = spawn_vehicle(world)
    setup_spectator(world, ego)

    ego.apply_control(carla.VehicleControl(brake=1.0))
    for _ in range(10):
        world.tick()

    # NPC vehicles
    tm = setup_traffic_manager(client, sync=True)
    npc_vehicles = spawn_npc_vehicles(world, tm, num_vehicles=30)

    # agent
    agent = BasicAgent(ego)
    spawn_points = world.get_map().get_spawn_points()

    ego_wp = world.get_map().get_waypoint(
        ego.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    while True:
        sp = random.choice(spawn_points)
        target_wp = world.get_map().get_waypoint(
            sp.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if target_wp and target_wp.road_id == ego_wp.road_id:
            target_location = target_wp.transform.location
            break

    agent.set_destination(target_location)
    print("[DEBUG] Agent destination set to:", target_location)

    print("[DEBUG] warming up BehaviorAgent planner...")
    for _ in range(10):
        world.tick()
    print("[DEBUG] planner warm-up done")

    # wheel
    wheel = None
    try:
        wheel = WheelController(config_path="wheel_config.ini")
        print("Wheel controller enabled")
    except Exception as e:
        print("Wheel controller not available:", e)

    # agent controller
    agent_controller = AgentController(agent, wheel)

    # setup camera
    camera = setup_camera(world, ego, width=1080, height=600)

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        rgb = array[:, :, :3][:, :, ::-1]  # BGRA → RGB

        surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        screen.blit(surface, (0, 0))
        pygame.display.flip()

    camera.listen(camera_callback)

    print("Simulation running... Ctrl+C to quit")

    try:
        while True:
            clock.tick(60)

            world.tick()  # synchronous mode

            mode_toggle.parse_events()

            # Debug info
            if int(world.get_snapshot().timestamp.elapsed_seconds) % 1 == 0:
                ego_loc = ego.get_location()
                ego_wp = world.get_map().get_waypoint(
                    ego_loc,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving
                )

            control = agent_controller.step(mode_toggle)
            ego.apply_control(control)

            follow_ego_spectator(world, ego)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Cleaning up actors...")
        ego.destroy()
        camera.stop()
        camera.destroy()
        pygame.quit()

if __name__ == "__main__":
    main()
