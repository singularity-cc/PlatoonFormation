import numpy as np


class BicycleVehiclePlant:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    """************************************CAV Simulated Physical Plant **********************************************"""

    def update(self):
        # print(f"cav id is {self.permanent_id}; target speed is {self.target_speed}, desired speed is {self.v_des}")
        # print(f"cav speed is {self.state.v}, acc is {self.input.acc}, heading is {self.state.heading}, steer is {self.input.steer_angle}")
        # print(f"cav location is {self.state.x}, {self.state.y}")
        self.update_kinematic()
        # if self.state.v <= 5:
        #     self.state.v = 0
        # else:
        #     self.update_lateral_state()
        #     self.update_longitudinal_state()
        # print(f"cav state: {self}")

    def update_motion(self):
        self.update_lateral_state()
        self.update_longitudinal_state()

    def update_kinematic(self):
        # if self.state.v <=
        phi = self.vehicle.state.heading / 180 * np.pi
        self.vehicle.state.x += self.vehicle.state.v * \
            self.vehicle.dt * np.cos(phi)
        self.vehicle.state.y += self.vehicle.state.v * \
            self.vehicle.dt * np.sin(phi)

        steer_angle = self.vehicle.input.steer_angle / 180 * np.pi
        self.vehicle.state.heading += (self.vehicle.state.v * np.tan(
            steer_angle) / self.vehicle.param.length * self.vehicle.dt) * 180 / np.pi
        self.vehicle.state.v += self.vehicle.input.acc * self.vehicle.dt

    def update_lateral_state(self):
        """Vehicle plant simulated by dynamic bicycle model"""
        g=9.81
        m=1500
        Iz=3000
        cf=38000
        cr=66000
        lf=2
        lr=3
        mju=0.02 # friction coefficient

        beta = self.vehicle.lateral_state.beta
        heading = self.vehicle.lateral_state.heading
        d_heading = self.vehicle.lateral_state.heading_dot
        delta = self.vehicle.input.steer_angle
        v = self.vehicle.state.v

        # the dynamics
        if v >= 2:
            d_beta = -(cf + cr) / (m * v) * beta + ((cr * lr - cf * lf) / (m * v**2) - 1) * d_heading + cf / (m * v) * delta
            dd_heading = - (cr * lr - cf * lf) / Iz * beta - (cr * lr**2 + cf * lf**2) / (Iz * v) * d_heading + cf * lf / Iz * delta
        else:
            d_beta = 0
            dd_heading = 0

        self.vehicle.lateral_state.beta += self.vehicle.dt * d_beta
        self.vehicle.lateral_state.heading += self.vehicle.dt * d_heading
        self.vehicle.lateral_state.heading_dot += self.vehicle.dt * dd_heading

        # update state heading
        self.vehicle.state.heading = self.vehicle.lateral_state.heading
        heading = self.vehicle.lateral_state.heading / 180 * np.pi
        beta = self.vehicle.lateral_state.beta / 180 * np.pi
        # update state positions
        self.vehicle.state.x += self.vehicle.dt * (v * np.cos(heading) - v * beta * np.sin(heading))
        self.vehicle.state.y += self.vehicle.dt * (v * np.sin(heading) + v * beta * np.cos(heading))

    def update_longitudinal_state(self):
        ca = 0.01
        cr = 0.001
        acc_des = self.vehicle.input.acc
        v = self.vehicle.state.v
        m = 1500

        acc = acc_des - (ca * v**2 + cr * v) / m
        self.vehicle.state.v += self.vehicle.dt * acc
