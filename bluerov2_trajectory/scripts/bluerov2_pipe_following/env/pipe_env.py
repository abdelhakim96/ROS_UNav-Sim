import gym
from gym import spaces
import numpy as np
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import ros_numpy
from matplotlib import pyplot as plt


class PipeEnv(gym.Env):
    def __init__(self):
        super(PipeEnv, self).__init__()
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(720, 1280, 3), dtype=np.uint8)

        self.image_observation = self.observation_space.sample()
        self.odom = Odometry()
        self.current_position = Odometry().pose.pose.position

        # ROS setup
        rospy.init_node('pipe_following_env')

        rospy.Subscriber("/airsim_node/RovSimple/front_right_custom/Scene", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/airsim_node/RovSimple/odom_local_ned", Odometry, self.odom_callback, queue_size=1)
        self.pub_position = rospy.Publisher("/ref_trajectory/position", Vector3, queue_size=1)
        self.pub_yaw = rospy.Publisher("/ref_trajectory/yaw", Float64, queue_size=1)

        self.step_length = 1
        self.step_counter = 0
        rospy.sleep(1)

    def step(self, action):
        # read action
        self.step_counter += 1
        forward_step = np.cos(action[0] * np.pi / 8) * self.step_length
        side_step = np.sin(action[0] * np.pi / 8) * self.step_length
        prev_pose = self.current_position
        # take action
        target_position = Vector3()
        target_position.x = prev_pose.x - forward_step
        target_position.y = prev_pose.y - side_step
        self.go_position(target_position)
        # take new observations
        rospy.sleep(0.1)
        # calculate reward
        reward = 0
        done = False
        if self.step_counter > 15:
            done = True
        info = {}
        # return
        return self.image_observation, reward, done, info

    def reset(self):
        self.step_counter = 0
        self.go_position(Vector3())
        return self.image_observation

    def go_position(self, target):
        np_target = self.point_to_numpy(target)
        np_position = self.point_to_numpy(self.current_position)
        distance = np.linalg.norm(np_target - np_position)
        while distance > 0.1:
            print("Vehicle moving, remaining distance:",distance)
            self.pub_position.publish(target)
            rospy.sleep(0.1)
            np_position = self.point_to_numpy(self.current_position)
            distance = np.linalg.norm(np_target - np_position)

    def image_callback(self, data):
        self.image_observation = ros_numpy.numpify(data)

    def odom_callback(self, data):
        self.odom = data
        self.current_position = self.odom.pose.pose.position

    def point_to_numpy(self, vec):
        return np.array([vec.x, vec.y, vec.z])

    def numpy_to_point(self, arr):
        p = Vector3()
        p.x, p.y, p.z = arr
        return p
