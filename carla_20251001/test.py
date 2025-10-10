import sys
sys.path.append('/home/kotei/huqianqian/software/Carla_0.9.15/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg')
import carla
client = carla.Client('localhost', 2000)
world = client.get_world()
print(world.get_map().name)