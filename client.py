import carla
import random
import pygame

from npc.generate_npc import spawn_npc_vehicles
from control.keyboard_control import KeyboardController

# client = carla.Client('localhost', 2000)
# client.set_timeout(10.0) 
# client.load_world('Town07') 
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

# 차량 스폰
def spawn_vehicle(world):
    blueprints = world.get_blueprint_library()
    # vehicle_bp = blueprints.filter('vehicle')[0]
    vehicle_bp = blueprints.filter('vehicle.tesla.model3')[0]
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
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

def main():
    pygame.init()
    screen = pygame.display.set_mode((1080, 600))
    screen.fill((30, 30, 30))
    pygame.display.flip()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)    # CARLA 서버 연결
    client.set_timeout(10.0)

    world = setup_world(client)  # 환경 세팅

    tm = setup_traffic_manager(client, sync=True, seed=42)

    # ego vehicle
    ego = spawn_vehicle(world)
    setup_spectator(world, ego)

    # NPC vehicles
    npc_vehicles = spawn_npc_vehicles(world, tm, num_vehicles=30)

    controller = KeyboardController(ego)

    print("Simulation running... Ctrl+C to quit")

    try:
        while True:
            world.tick()
            controller.tick(clock)
            clock.tick(60)
    finally:
        print("Cleaning up actors...")
        ego.destroy()
        for v in npc_vehicles:
            v.destroy()
        pygame.quit()


if __name__ == "__main__":
    main()
