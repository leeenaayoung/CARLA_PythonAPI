import random
import carla

# NPC 차량 스폰 함수
def spawn_npc_vehicles(
    world,
    tm,
    num_vehicles=20,
    vehicle_filter="vehicle.*"
):
    blueprints = world.get_blueprint_library().filter(vehicle_filter)
    spawn_points = world.get_map().get_spawn_points()

    random.shuffle(spawn_points)
    vehicles = []

    for transform in spawn_points[:num_vehicles]:
        bp = random.choice(blueprints)
        bp.set_attribute("role_name", "autopilot")

        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle:
            vehicle.set_autopilot(True, tm.get_port())
            vehicles.append(vehicle)

    return vehicles


