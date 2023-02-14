## Author Hakim Amer
## Date: 28/10/2022

from scipy.spatial.transform import Rotation as R
from bluerov2_pipe_following.bluerov_mpc import setup_path
import airsim
import cv2
import threading
import time
import numpy as np
import quaternion
import math
import acado
import sys, os
import argparse

sys.path.append(os.path.join(os.path.dirname(sys.path[0]), 'ccode'))  # path to c-code


# solver for optimal control.


class ControlInput(object):  # define control input
    def __init__(self, F_X, F_Y, F_Z, M_Z):
        self.F_X = F_X
        self.F_Y = F_Y
        self.F_Z = F_Z
        self.M_Z = M_Z


class MPC(object):
    def __init__(self, x_ref=0.0, y_ref=0.0, z_ref=2.0, horizon=1):
        # Iteration for approximating mpc. 
        self.max_iteration = 15000
        # Cost of mpc.  
        # TODO: make weights parameters read from a config file 
        #self.Q = np.diag( [30.0, 30.0, 10.0, 0, 0, 0, 1.0, 0.5, 0.1, 0.1, 0.1, 1.0])  # [x,y,z,u,v,w,psi,r,F_X,F_Y,F_Z,M_Z] old

        self.Q = np.diag(
            [30.0, 30.0, 30.0, 0.0, 0.0, 10.0, 1.0, 1.0, 0.1, 0.1, 0.1, 1.0])  # [x,y,z,u,v,w,psi,r,F_X,F_Y,F_Z,M_Z]
        self.Qf = self.Q[:8, :8]
        # Step size and horizon
        #self.horizon = 20
        self.horizon = 100
        # Copy from acado.
        # TODO: make number of variables read from acado c code
        self.NX = 8
        self.NY = 12
        self.NYN = 8
        self.NU = 4
        self.reference = np.array(
            [-6.2, -5.6, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x,y,z,u,v,w,psi,r,F_X,F_Y,F_Z,M_Z]
        self.THRESHOLD = 0.001
        self.use_ith_control = 1
        self.g = 9.81

        # airsim API
        self.client = airsim.RovClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.drone_state = self.getState()

    # set the reference of the MPC
    def setReference(self, current):
        # assert(self.NY == self.reference.shape[0])
        yn = np.zeros((1, self.NYN))
        Y = np.zeros((self.horizon, self.NY))
        yn[0, :] = self.reference[:1]
        Y[:, :] = self.reference
        # Y[:,:] = self.reference[:self.horizon]

        return (Y, yn)

    ##OCP stuff
    ## create state feeback vector for acado

    # Get input and prediction along prediction horizon
    def getFullMpcOutput(self, state):
        # finish iter
        X = np.zeros((self.horizon + 1, self.NX))
        U = np.zeros((self.horizon, self.NU))
        prev_x = np.zeros((1, self.NX))
        ref_traj, terminal_state = self.setReference(state)
        # print("psi_ref", ref_traj[0, 6])
        # print("psi", state[0, 6])
        for i in range(self.max_iteration):
            X, U = acado.mpc(0, 1, state, X, U, ref_traj, terminal_state, np.transpose(np.tile(self.Q, self.horizon)),
                             self.Qf, 0)
            if (np.linalg.norm(X - prev_x) < self.THRESHOLD):
                # print("CONTROL: Input mpc terminating iteration at ", i)
                break
            prev_x = X  # Update prev
        return (X, U)

        # Get the first input, which is to be applied by the MPC

    def getInput(self, state):
        X, U = self.getFullMpcOutput(state)
        F_X = U[1, 0]
        F_Y = U[1, 1]
        F_Z = -U[1, 2]
        M_Z = U[1, 3]

        #print('forces:', F_X, F_Y, F_Z)
        return ControlInput(F_X, F_Y, F_Z, M_Z)

    def getState(self):
        MRotor_state = self.client.getRovState().kinematics_estimated  # AirSim API callback to get the multotor state
        pitch = airsim.to_eularian_angles(MRotor_state.orientation)[0]  # get euler angles
        roll = airsim.to_eularian_angles(MRotor_state.orientation)[1]
        yaw = airsim.to_eularian_angles(MRotor_state.orientation)[2]
        v_w = np.zeros((3, 1))
        v_b = np.zeros((3, 1))
        drone_state = np.zeros((1, 8))
        v_w[0, 0] = MRotor_state.linear_velocity.x_val
        v_w[1, 0] = MRotor_state.linear_velocity.y_val
        v_w[2, 0] = MRotor_state.linear_velocity.z_val

        v_b[0, 0] = np.cos(yaw) * v_w[0, 0] + np.sin(yaw) * v_w[1, 0]
        v_b[1, 0] = -np.sin(yaw) * v_w[0, 0] + np.cos(yaw) * v_w[1, 0]

        v_b[2, 0] = v_w[2, 0]

        drone_state[0, 0] = MRotor_state.position.x_val
        drone_state[0, 1] = MRotor_state.position.y_val
        drone_state[0, 2] = MRotor_state.position.z_val
        drone_state[0, 3] = v_b[0, 0]
        drone_state[0, 4] = v_b[1, 0]
        drone_state[0, 5] = v_b[2, 0]
        drone_state[0, 6] = yaw
        drone_state[0, 7] = MRotor_state.angular_velocity.z_val

        #print("x : ", drone_state[0, 0])
        #print("y : ", drone_state[0, 1])
        # print("z : ", drone_state[0, 2])
        # print("yaw", drone_state[0,6])
        # print("r", drone_state[0,7])
        # print("yaw_rate", drone_state[0,7])
        return drone_state

    ## send control commands to AirSim
    def apply_control(self):
        self.drone_state = self.getState()
        input = self.getInput(self.drone_state)  # apply control input
        # print("Wrench (X, Y, Z)", input.F_X, input.F_Y, input.F_Z, input.M_Z)
        A = np.array([[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, -0.5],
                      [0.0, 0.0, 0.0, 0.0, -0.5, 0.5, 0.5, -0.5],
                      [1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.1, -0.1, 0.1, -0.1]])  # Array of floats
        #A = np.array([[0.0, 0.0, 0.0, 0.0, 10.0, 10.0, -10.0, -10.0],
        #              [0.0, 0.0, 0.0, 0.0, -10.0, 10.0, 10.0, -10.0],
        #              [1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        #              [0.0, 0.0, 0.0, 0.0, 0.1, -0.1, 0.1, -0.1]])  # Array of floats

        # A = np.array([[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, -0.5], [0.0, 0.0, 0.0, 0.0, -0.5, 0.5,0.5,-0.5],[1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]]) # Array of floats
        B = np.linalg.pinv(A)
        v = [[input.F_X], [input.F_Y], [input.F_Z], [-input.M_Z]]
        # v=[ [input.F_X], [input.F_Y], [input.F_Z], [0.0]]
        C = B @ v
        # print(self.drone_state[0, 0:2], self.drone_state[0, 6])
        Tz = 0.0
        Tx = -0.3

        self.client.moveByMotorPWMsAsync([C[0][0], C[1][0], C[2][0], C[3][0], C[4][0], C[5][0], C[6][0], C[7][0]],
                                         0.001)
        # self.client.moveByMotorPWMsAsync([1.0,1.0,1.0,1.0,-0.0,0.0,0.0,-0.0],0.001)


if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser(
        "mpc_control.py makes mpc controller for waypoint following: Please specify waypoints")
    arg_parser.add_argument("--x_ref", type=float, help="reference x coordinate", default=2.0)
    arg_parser.add_argument("--y_ref", type=float, help="reference x coordinate", default=2.0)
    arg_parser.add_argument("--z_ref", type=float, help="reference x coordinate", default=2.0)
    args = arg_parser.parse_args(args)
    mpc_control = MPC(args.x_ref, args.y_ref, args.z_ref, 20)
    # mpc_control.reference = np.array(
    #         [0.0, 3.0, 0.0, 0.0, 0.0, 0.0, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0])
    while True:
        mpc_control.apply_control()
