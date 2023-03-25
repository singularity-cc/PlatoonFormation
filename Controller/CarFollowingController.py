from Vehicle import *
import numpy as np

class CarFollowingController:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def update(self):
        self.update_car_following_decision()


    def update_car_following_decision(self):
        """If vehicle is in lane-change state"""
        if self.vehicle.is_in_lane_change:
            self.vehicle.input.acc = 0
            return 

        """If vehicle is not lane-change, use IDM car-following model"""
        self.vehicle.input.acc = self.IDM_car_following()



    def IDM_car_following(self):
        """Intelligent Driving Model to simulate HDV car-following maneuvers"""
        v = self.vehicle.state.v
        T = 1.5
        a = 0.73
        b = 1.67
        delta = 4
        l = self.vehicle.param.length
        s0 = 2 + l # buffer distance will count vehicle length
        x = self.vehicle.state.x
        y = self.vehicle.state.y
        # print(f"id is: {self.id}")
        if self.vehicle.id == 0:
            s = 100000 # s is net distance: x_a-1 - x_a - l_a-1: if there is no preceding vehicle
            dv = 0.0 # dv is speed difference between preceding vehicle and itself: dv = v_a - v_a-1
        else:
            precede_veh = self.vehicle.segment.vehicles[self.vehicle.lane][self.vehicle.id - 1]
            s = np.sqrt((precede_veh.state.x - x)**2 + (precede_veh.state.y - y)**2) - l # why 1 is too small
            dv = v - precede_veh.state.v
        s_des = s0 + v * T + v * dv / (2 * np.sqrt(a * b))

        # if s < l + T * v - v ** 2 / self.acc_min:
        #     acc = self.acc_min
        #     acc = max(acc, (self.v_min - v) / self.dt)
            # if s < l + 5:
            #     acc = -7
        # IDM equation
        if self.vehicle.v_des <= 0 or s <= 0:
            acc = 0
        elif v < 0:
            acc = self.vehicle.acc_max
        else:
            acc = a * (1 - (v / self.vehicle.v_des) ** delta - (s_des / s) ** 2)
            acc = min(acc, self.vehicle.acc_max)
            acc = max(acc, self.vehicle.acc_min)
            acc = min(acc, (self.vehicle.v_max - v) / self.vehicle.dt)
            acc = max(acc, (self.vehicle.v_min - v) / self.vehicle.dt)
        return acc