from .Controller import *
from Utilities import *
import math

class LonLatController:
    def __init__(self, cav):
        self.cav = cav
        self.init_control_module()

    def init_control_module(self):
        
        # pid configuration
        self.cav.match_point_idx = 0
        self.pid_dp = 0
        self.pid_di = 0
        self.pid_dd = 0

        # pure pursuit configuration
        if self.cav.state.v >= 25:
            self.k = 0.1
        elif self.cav.state.v >= 20:
            self.k = 0.3
        elif self.cav.state.v >= 15:
            self.k = 1
        else:
            self.k = 2
        self.look_ahead_angle = 0
        self.look_ahead_distance = self.k * self.cav.state.v if self.cav.state.v >= 10 else 5
        self.look_ahead_point = Point(2, 2)

    def update(self):
        if self.cav.state.v <= 0:
            self.cav.input.steer_angle = 0
            self.cav.input.acc = self.longitudinal_control()
        else:
            self.cav.match_point_idx = self.find_match_point()
            self.cav.v_des = self.cav.discrete_speed_reference[self.cav.match_point_idx]
            self.look_ahead_angle = self.find_look_ahead_angle()
            # print(f"v_des: {self.v_des}")
            self.longitudinal_lateral_decomposed_control()

    """******************************************CAV Control Algorithm*****************************************"""
    def find_match_point(self):
        # TODO: find the match point of the trajectory reference and use controller to track the speed of it
        idx = 0
        min_distance = 100000
        for i in range(len(self.cav.discrete_path_reference)):
            ego_point = self.cav.point_location()
            point = self.cav.discrete_path_reference[i]
            dis = distance(ego_point, point)
            if dis < min_distance:
                min_distance = dis
                idx = i
        self.cav.match_point_idx = idx
        # print(f"match point idx is found at {self.cav.match_point_idx}")
        return idx

    def find_look_ahead_angle(self):

        look_ahead_point = self.find_look_ahead_point()
        ego_point = self.cav.point_location()
        # print(f"first look ahead point is {look_ahead_point}")
        # print(f"first ego point is {ego_point}")

        look_ahead_vec = Vector(self.look_ahead_point.x - ego_point.x, self.look_ahead_point.y - ego_point.y)#.normalize()

        heading_rad = self.cav.state.heading / 180 * np.pi
        heading_vec = Vector(np.cos(heading_rad), np.sin(heading_rad))

        return angle_between_vectors_with_sign(heading_vec, look_ahead_vec)

    def find_look_ahead_point(self):
        # look_ahead_distance = self.k * self.cav.state.v
        ego_point = self.cav.point_location()

        look_ahead_point_idx = self.cav.match_point_idx
        min_distance = 10000
        # search the min distance point in the trajectory path points
        for i in range(self.cav.match_point_idx ,len(self.cav.discrete_path_reference)):
            point = self.cav.discrete_path_reference[i]
            dis = distance(point, ego_point)
            if np.abs(dis - self.look_ahead_distance) < min_distance:
                min_distance = np.abs(dis - self.look_ahead_distance)
                look_ahead_point_idx = i

        self.look_ahead_point = self.cav.discrete_path_reference[look_ahead_point_idx]
        return self.cav.discrete_path_reference[look_ahead_point_idx]


    def longitudinal_lateral_decomposed_control(self):
        self.cav.input.acc = self.longitudinal_control()
        if self.cav.state.v <= 4:
            self.cav.input.steer_angle = 0
        else:
            self.cav.input.steer_angle = self.lateral_control()

    def longitudinal_control(self):
        kp = 0.2
        ki = 0.05
        kd = 0.01

        self.pid_dd = -(self.pid_dp - (self.cav.v_des - self.cav.state.v)) / self.cav.dt
        self.pid_dp = self.cav.v_des - self.cav.state.v
        self.pid_di += self.pid_dp * self.cav.dt

        # clear the accummulated pi term every 2.5 seconds
        if self.cav.simulation.count % 250 == 0:
            self.pid_di = 0
        
        # print(f"pid_dp is {self.pid_dp}")
        # print(f"pid_dd is {self.pid_dd}")
        # print(f"pid_di is {self.pid_di}")

        acc = kp * self.pid_dp + ki * self.pid_di + kd * self.pid_dd
        # print(f"acc is {acc}")
        acc = max(min(acc, 3), -4)
        return acc
        

    def lateral_control(self):
        # print(f"angle: {self.look_ahead_angle}")
        # print(f"turn angle: {np.arctan2(2 * self.cav.param.length * self.look_ahead_angle, self.look_ahead_distance)}")
        if self.cav.state.v <= 0.001:
            return 0
        # print(f"match point idx is {self.cav.match_point_idx}, length of path reference is {len(self.cav.discrete_path_reference)}")
        # print(f"look ahead angle is {self.look_ahead_angle}, look ahead point is {self.look_ahead_point}")
        if math.isnan(self.look_ahead_angle):
            return 0
        angle = np.arctan2(2 * self.cav.param.length * self.look_ahead_angle, self.look_ahead_distance) * 180 / np.pi
        # angle = min(max(angle, -0.5), 0.5)
        return angle

