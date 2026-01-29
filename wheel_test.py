import carla
import random
import time

from agents.navigation.behavior_agent import BehaviorAgent

def follow_vehicle(world, vehicle, height=3.0, pitch=-15):
    spectator = world.get_spectator()
    transform = vehicle.get_transform()

    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(z=height),
            carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw)
        )
    )

def main():
    # 1. CARLA 서버 연결
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    carla_map = world.get_map()

    # 2. synchronous mode (권장)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 3. 차량 스폰
    blueprint = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    spawn_point = random.choice(carla_map.get_spawn_points())
    vehicle = world.spawn_actor(blueprint, spawn_point)

    # 4. BasicAgent 생성
    agent = BehaviorAgent(vehicle, behavior='normal')

    # 5. 목적지 설정 (같은 road 상의 다른 spawn point)
    destination = random.choice(carla_map.get_spawn_points()).location
    agent.set_destination(destination)

    print("[INFO] BasicAgent started")

    try:
        while True:
            world.tick()
            control = agent.run_step()
            vehicle.apply_control(control)

            if agent.done():
                print("[INFO] Destination reached")
                break

            time.sleep(0.05)
            follow_vehicle(world, vehicle) 

    finally:
        vehicle.destroy()
        print("[INFO] Vehicle destroyed")


if __name__ == "__main__":
    main()
