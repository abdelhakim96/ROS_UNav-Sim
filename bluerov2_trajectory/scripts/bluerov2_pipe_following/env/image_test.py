from matplotlib import pyplot as plt
import airsim
import numpy as np
from PIL import Image
from io import BytesIO

from bluerov2_pipe_following.bluerov_mpc.mpc_bluerov2 import MPC
from bluerov2_pipe_following.env.pipe_env_api import PipeEnv

env = PipeEnv()
env.reset()
for i in range(5):
    env.step([1, 0])

client = airsim.RovClient()
im = client.simGetImage('front_right_custom', 0)
arr = np.array(Image.open(BytesIO(im)))

plt.imshow(arr)
plt.show()

