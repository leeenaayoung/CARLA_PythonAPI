# CARLA 서버 연결
import carla
import random

client = carla.Client('localhost', 2000)

# 새로운 도시 불러오기, 시뮬레이션 기록
# client.load_world('Town07') 
client.start_recorder('recording.log')

# 시뮬레이션을 나타내는 객체 불러오기(한 시뮬레이션 당 하나의 월드만 존재)
world = client.get_world()

# 디버깅
print("Connected to:", world.get_map().name)

# 월드의 환경 구성요소 접근
level = world.get_map()
weather = world.get_weather()
blueprints = world.get_blueprint_library()
for bp in blueprints.filter('vehicle'):
    print(bp.id)
    break

vehicle_bp = blueprints.filter('vehicle')[0]

# random 차량 스폰
spawn_point = random.choice(world.get_map().get_spawn_points())
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

spectator = world.get_spectator()
transform = vehicle.get_transform()

spectator.set_transform(
    carla.Transform(
        transform.location + carla.Location(z=10),
        carla.Rotation(pitch=-30)
    )
)

print("Vehicle spawned:", vehicle.type_id)

input("Press Enter to quit")
vehicle.destroy()
