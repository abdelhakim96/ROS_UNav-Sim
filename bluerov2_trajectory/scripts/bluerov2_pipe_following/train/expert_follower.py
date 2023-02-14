from bluerov2_pipe_following.env.pipe_env_api import PipeEnv
import numpy as np

env = PipeEnv()
env.reset()
way_points = [
    [[-4.5, 0, 0], [+np.pi/2]],
    [[-4.5, -5.5, 0], [0]],
    [[-13.5, -5.5, 0], [0]],
]
for point in way_points:
    pos, yaw = point
    # print(pos, yaw[0])
    env.go_position(pos, yaw[0])
    im = env.get_image_array()