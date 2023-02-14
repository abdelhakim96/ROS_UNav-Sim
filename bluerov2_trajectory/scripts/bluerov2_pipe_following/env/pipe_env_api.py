import gym
from gym import spaces
import numpy as np
from matplotlib import pyplot as plt
import airsim
import numpy as np
from PIL import Image
from io import BytesIO
from bluerov2_pipe_following.bluerov_mpc.mpc_bluerov2 import MPC
from bluerov2_pipe_following.utils.plot_utils import plot_image_array


class PipeEnv(gym.Env):
    def __init__(self):
        super(PipeEnv, self).__init__()
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(180, 320, 3), dtype=np.uint8)

        self.image_observation = self.observation_space.sample()
        # self.odom = Odometry()
        # self.current_position = Odometry().pose.pose.position

        # AirSim setup
        self.client = airsim.RovClient()
        self.mpc_control = MPC()
        self.mpc_control.reference = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x,y,z,u,v,w,psi,r,F_X,F_Y,F_Z,M_Z]

        self.step_length = 1
        self.step_counter = 0
        self.prev_pos, self.prev_yaw = self.get_pose()

    def step(self, action):
        # read action
        # print("step taken")
        # print(action)
        self.step_counter += 1
        forward_step = np.cos(action[0] * np.pi / 8) * self.step_length
        side_step = np.sin(action[0] * np.pi / 8) * self.step_length
        # prev_pos, prev_yaw = self.get_pose()
        # take action
        x_step = forward_step * np.cos(self.prev_yaw) - side_step * np.sin(self.prev_yaw)
        y_step = forward_step * np.sin(self.prev_yaw) + side_step * np.cos(self.prev_yaw)
        yaw_step = action[1] * np.pi / 8
        target_position = np.array([self.prev_pos[0] - x_step, self.prev_pos[1] - y_step, 0])
        target_yaw = np.clip(self.prev_yaw + yaw_step, -3*np.pi/4, 3*np.pi/4)
        reward, done = self.go_position(target_position, target_yaw)
        # take new observations
        self.image_observation = self.get_image_array()
        # calculate reward
        # if self.step_counter > 15:
        #     done = True
        info = {"position": self.mpc_control.drone_state[0, 0:3], "yaw": self.mpc_control.drone_state[0, 6]}

        # print(self.prev_pos, self.prev_yaw)
        # return
        # print(info["position"])
        # plot_image_array(self.image_observation)

        return self.image_observation, reward, done, info

    def reset(self):
        print("RESET")
        self.step_counter = 0
        if np.random.choice(2) == 1:
            reset_target_pos = np.array([-4.2, 0.0, 0.0])
            reset_target_yaw = np.pi/2
        else:
            reset_target_pos = np.array([0.0, 0.0, 0.0])
            reset_target_yaw = 0
        distance = np.linalg.norm(self.prev_pos[0:2] - reset_target_pos[0:2])

        while distance > 1:
            target = self.prev_pos[0:2] - (self.prev_pos[0:2] - reset_target_pos[0:2])/distance
            self.go_position(np.array([target[0], target[1], 0]), target_yaw=reset_target_yaw)
            distance = np.linalg.norm(self.prev_pos[0:2] - reset_target_pos[0:2])
        self.go_position(reset_target_pos, target_yaw=reset_target_yaw, yaw_threshold=0.2)
        self.image_observation = self.get_image_array()
        print(self.prev_pos, self.prev_yaw)
        return self.image_observation

    def go_position(self, target_pos, target_yaw=0, threshold=0.5, yaw_threshold=5):  # target is array of 3: x, y, z position
        pos_error = 1
        yaw_error = 1
        self.mpc_control.reference[0:3] = target_pos
        self.mpc_control.reference[6] = target_yaw
        while pos_error > threshold or yaw_error > yaw_threshold:
            self.mpc_control.apply_control()
            self.prev_pos = self.mpc_control.drone_state[0, 0:3]
            self.prev_yaw = self.mpc_control.drone_state[0, 6]
            pos_error = np.linalg.norm(target_pos[0:2] - self.prev_pos[0:2])
            yaw_error = np.abs(target_yaw - self.prev_yaw)
            # print("Vehicle moving, remaining distance:", distance, "Target and pose", target, self.position)
        self.prev_pos = np.array(self.mpc_control.drone_state[0, 0:3])
        self.prev_yaw = self.mpc_control.drone_state[0, 6]
        return self.calculate_reward(self.prev_pos[0], self.prev_pos[1])

    def calculate_reward(self, px, py):
        side_threshold = 2.5
        pipe_break_x = 4.2
        pipe_break_y = 5.6
        if px + py > -pipe_break_x:  # region one
            # print("region one")
            if np.abs(py) > side_threshold or px > 0:
                return -20, True
            return 10 - 2*(np.abs(py)**2) - 2*np.abs(self.prev_yaw), False
        if px + py > -(pipe_break_x+pipe_break_y):  # region two
            # print("region two")
            if np.abs(px + pipe_break_x) > side_threshold:
                return -20, True
            return 10 - 2*(np.abs(px + pipe_break_x)**2) - 2*np.abs(self.prev_yaw - np.pi/2), False
        # print("region three")
        if px < -13:
            return 25, True
        if np.abs(py+pipe_break_y) > side_threshold:
            return -20, True
        return 10 - 2*(np.abs(py+pipe_break_y) ** 2) - 2*np.abs(self.prev_yaw), False


    def get_image_array(self):
        im = self.client.simGetImage('front_right_custom', 0)
        while not isinstance(im, bytes):
            print("Warning: image is notbytes")
            im = self.client.simGetImage('front_right_custom', 0)
        while np.sum(np.array(Image.open(BytesIO(im)))[:, :, 0:3]) == 0:
            print("Warning: blank image")
            im = self.client.simGetImage('front_right_custom', 0)
        im = Image.open(BytesIO(im))
        im = im.rotate(180)   # fix the camera rotation
        im = im.resize((320, 180), Image.ANTIALIAS)
        return np.array(im)[:, :, 0:3]

    def get_pose(self):
        state = self.client.getRovState().kinematics_estimated
        pos = [state.position.x_val, state.position.y_val, state.position.z_val]
        yaw = airsim.to_eularian_angles(state.orientation)[2]
        return np.array(pos), yaw
