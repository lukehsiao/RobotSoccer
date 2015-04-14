import time

class State:
    def __init__(self):
        self.time = 0.0
        self.pos_x_est = 0.0
        self.pos_x_des = 0.0
        self.pos_y_est = 0.0
        self.pos_y_des = 0.0
        self.pos_theta_est = 0.0
        self.pos_theta_des = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.omega = 0.0
        self.kill = 0.0

    def getUpdatedState(state,toTime):
        result = State()
        result.time = toTime
        deltaTime = toTime - state.time
        result.pos_x_est = state.pos_x_est + deltaTime * state.vel_x
        result.pos_x_des = state.pos_x_des + deltaTime * state.vel_x
        result.pos_y_est = state.pos_y_est + deltaTime * state.vel_y
        result.pos_y_des = state.pos_y_des + deltaTime * state.vel_y
        result.pos_theta_est = state.pos_theta_est + deltaTime * state.omega
        result.pos_theta_des = state.pos_theta_des + deltaTime * state.omega
        return result

class Kalman:
    def __init__(self):
       self.Qx = .1
       self.Qy = .1
       self.Qtheta = .1
       self.Sx = 1.0
       self.Sy = 1.0
       self.Stheta = 1.0
       self.Rx = 0.015
       self.Ry = 0.015
       self.Rtheta = 0.09
       self.Lx = 0.0
       self.Ly = 0.0
       self.Ltheta = 0.0

class Robot:
    def __init__(self):
        self.init_sample = 0
        self.isInitialized = False
        self.state = State()
        self.kalman = Kalman()