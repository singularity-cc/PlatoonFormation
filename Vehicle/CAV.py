from .Vehicle import *
from Utilities import *
from collections import deque
import numpy as np
import datetime 


class MotionState:
    def __init__(self, s, l, v):
        self.s = s
        self.l = l
        self.v = v

    def __str__(self):
        return f"s is {self.s}, l is {self.l}, v is {self.v}"

class LateralState:
    def __init__(self, heading = 0, beta = 0, heading_dot = 0):
        self.beta = beta
        self.heading = heading
        self.heading_dot = heading_dot


class CAV(Vehicle):

    def __init__(self, dt, simulation, v_des, permanent_id, lane, id, veh_state, veh_param, veh_input = VehicleInput(0, 0)):
        # we need some label to index the HDV lane, id....
        super().__init__(dt, simulation, v_des, permanent_id, lane, id, veh_state, veh_param, veh_input)

        self.category = "CAV"
        self.lateral_state = LateralState(heading = veh_state.heading)

        # computation time summary
        self.control_compute_time = []
        self.motion_plan_compute_time = []
        self.target_compute_time = []

        # target module
        self.target_location = None
        self.target_speed = None

        # initialize motion planner configuration
        

        # initialize controller configuration
        self.discrete_path_reference = []
        self.discrete_speed_reference = []
        self.control_track_point = None

        # pid configuration
        self.match_point_idx = 0
        self.pid_dp = 0
        self.pid_di = 0
        self.pid_dd = 0

        # pure pursuit configuration
        self.k = 0.1
        self.look_ahead_angle = 0
        self.look_ahead_point = Point(2, 2)

        # add temp target location and speed at the end of the segment
        self.target_location = self.segment.end.convert_to_point()
        self.target_speed = 10
        self.target_heading = 0

        self.trajectory_set = []
        self.path_set = deque()
        self.speed_set = deque()
        self.visualization_set = []

        self.path_edges = []
        self.speed_edges = []

        self.best_path = []
        self.best_speed = []

        self.dense_visualization_set = []

        self.trajectory_set_with_dense_info = []
    

        self.num_trajectory_points = 500
        # update speed trajectory reference
        v = 20
        # for i in range(5000):
        #     self.discrete_speed_reference.append(v)
        # print("v")
        for i in range(100):
            v += -0.02
            self.discrete_speed_reference.append(v)
        for i in range(100):
            v -= 0.00
            self.discrete_speed_reference.append(v)
        for i in range(300):
            self.discrete_speed_reference.append(v)

        if self.segment.is_segment_straight_getter():
            start = Point(self.segment.start.x, self.segment.start.y)
            end = Point(self.segment.end.x, self.segment.end.y)
            for i in range(500):
                x = ((500 - i) * start.x + i * end.x) / 500
                y = ((500 - i) * start.y + i * end.y) / 500
                # print(x)
                self.discrete_path_reference.append(Point(x, y))
        # if segment is not straight
        else:
            arc_start_angle = self.segment.arc_start_angle
            arc_end_angle = self.segment.arc_end_angle
            arc_center = self.segment.arc_center
            arc_radius = self.segment.arc_radius

            for i in range(500):
                angle = ((500 - i) * arc_start_angle + i * arc_end_angle) / 5000
                rad = angle * np.pi / 180.0
                angle_vec = Vector(np.cos(rad), np.sin(rad))
                center_to_point_vec = angle_vec * arc_radius
                point = Point(arc_center.x + center_to_point_vec.x, arc_center.y + center_to_point_vec.y)
                self.discrete_path_reference.append(point)


    def update(self):
        
        self.update_target()
        self.update_trajectory_reference()
        self.update_control_command()
        self.update_state()


    def update_target(self):
        self.target_location = self.point_location() + Point(150, 0)

    def update_trajectory_reference(self):
        if self.simulation.count % 10 == 0:
            start_time = datetime.datetime.now()
            self.generate_trajectory_set()
            self.select_best_path()
            # self.display_trajectory()

            end_time = datetime.datetime.now()
            self.motion_plan_compute_time.append((end_time - start_time).microseconds / 1000)
            print(f"Motion computation time: {(end_time - start_time).microseconds / 1000}")

    def display_trajectory(self):
        for trajectory in self.trajectory_set:
            for i in range(len(trajectory)):
                print(f"Trajectory at {i} is {trajectory[i]}")

    def update_control_command(self):
        start_time = datetime.datetime.now()
        self.match_point_idx = self.find_match_point()
        self.v_des = self.discrete_speed_reference[self.match_point_idx]
        self.look_ahead_angle = self.find_look_ahead_angle()
        # print(f"v_des: {self.v_des}")
        self.longitudinal_lateral_decomposed_control()
        end_time = datetime.datetime.now()
        self.control_compute_time.append((end_time - start_time).microseconds / 1000)
        print(f"Control computation time: {(end_time - start_time).microseconds / 1000}")



    """******************************************CAV motion trajectory Algorithm*****************************************"""

    def generate_trajectory_set(self):
        self.num_layers = 5
        self.num_path_samples_per_layer = 3 # can only focus on the adjacent lanes
        self.num_speed_samples_per_layer = 5

        self.vmin = 0
        self.vmax = 30

        self.generate_path_set()
        # self.generate_speed_set()
        # self.combine_path_speed()
        self.update_visualization_set()



    def select_best_trajectory(self):
        self.coordinate_with_other_CAVs()
        self.best_trajectory = self.select_best_trajectory_according_to_cost()

    def improve_best_trajectory(self):
        # ToDo: iteratively improve path and speed trajectory
        self.improve_path()
        self.improve_speed()

    """trajectory generation"""
    def select_best_path(self):
        # Estimate the best path assuming speed is constant in the future short time period
        

        pass

    def generate_path_set(self):
        # for straight road
        self.path_samples = [[] for _ in range(self.num_layers)]

        # add initial state at first
        self.path_samples[0].append(Point(0,0))
        # add middle layer states
        # temp on straight road scenarios
        longitudinal_distance = distance(self.target_location, self.point_location())
        lateral_distance = self.road.num_lanes * self.road.lane_width
        for i in range(1, self.num_layers - 1):
            frenet_s = longitudinal_distance / (self.num_layers - 1) * i
            for j in range(self.road.num_lanes):
                frenet_l = -self.road.lane_width + j * self.road.lane_width
                self.path_samples[i].append(Point(frenet_s, frenet_l))

        # add target state at end
        self.path_samples[-1].append(Point(longitudinal_distance, 0))
        # print(f"longitudinal distance: {longitudinal_distance}")

        self.path_edges.clear()
        #a0 =l0, a1 = 0, 2 * a2 + 3 * a3 * s = 0; a2 + a3 * s = (l1 - l0) / s**2 ==> a3 = -2 * (l1 - l0) / s**3, a2 = 3 * (l1 - l0) / s**2
        
        num_samples = 5
        for i in range(self.num_layers - 1):
            path_edges_at_layer = []
            for start_point in self.path_samples[i]:
                for end_point in self.path_samples[i + 1]:
                    sample_points_at_one_edge = []
                    s = end_point.x - start_point.x
                    l = end_point.y - start_point.y
                    s_polynomial = [0, 0, 3 * l / s**2, -2 * l / s**3]

                    ds = s / num_samples 
                    for j in range(num_samples):
                        s_sample = ds * j
                        l_sample = s_polynomial[0] + s_sample * (s_polynomial[1] + s_sample * (s_polynomial[2] + s_sample * s_polynomial[3]))
                        sample_points_at_one_edge.append(Point(s_sample + start_point.x, l_sample + start_point.y))
                        # print(f"{i}: {Point(s_sample, l_sample)}")

                    path_edges_at_layer.append(sample_points_at_one_edge)
            # print(f"{i}: {path_edges_at_layer}")
            self.path_edges.append(path_edges_at_layer)

        # print(len(self.path_edges))

        # generate path set using breadth first search
        self.path_set.clear()
        self.path_set.append(self.path_samples[0].copy())

        for layer in range(1, self.num_layers):
            num_size = len(self.path_set)
            for i in range(num_size):
                path_list = self.path_set[0].copy()
                self.path_set.popleft()
                for path in self.path_samples[layer]:
                    new_path_list = path_list[:]
                    new_path_list.append(path)
                    self.path_set.append(new_path_list)

        # connect edges using polynomials
        # print("path stop")


    def select_best_path(self):
        
        pass

    def update_visualization_set(self):
        self.visualization_set.clear()
        self.dense_visualization_set.clear()

        for layer in self.path_samples:
            for sample in layer:
                point = sample + self.point_location()
                self.visualization_set.append(point)

        for path_samples_at_layer in self.path_edges:
            # print(len(path_samples_at_layer))
            for sample_points_at_one_edge in path_samples_at_layer:
                # print(len(sample_points_at_one_edge))
                for sample in sample_points_at_one_edge:
                    point = sample + self.point_location()
                    self.dense_visualization_set.append(point)

        # for dense_trajectory in self.trajectory_set_with_dense_info:
        #     for state in dense_trajectory:
        #         point = Point(state[0], state[1]) + self.point_location()
        #         self.dense_visualization_set.append(point)

    def generate_speed_set(self):
        self.speed_samples = [[] for _ in range(self.num_layers)]

        # add initial speed
        self.speed_samples[0].append(self.state.v)

        # add middle layer speeds
        for i in range(1, self.num_layers - 1):
            for j in range(self.num_speed_samples_per_layer):
                v = self.v_min + (self.v_max - self.v_min) / self.num_speed_samples_per_layer * j
                self.speed_samples[i].append(v)

        # add target speed
        self.speed_samples[-1].append(self.target_speed)


        # generate speed set using breadth first search
        self.speed_set.clear()
        self.speed_set.append(self.speed_samples[0].copy())

        for layer in range(1, self.num_layers):
            num_size = len(self.speed_set)
            for i in range(num_size):
                speed_list = self.speed_set[0].copy()
                self.speed_set.popleft()
                for speed in self.speed_samples[layer]:
                    new_speed_list = speed_list[:]
                    new_speed_list.append(speed)
                    self.speed_set.append(new_speed_list)

        # print("speed stop")

    def combine_path_speed(self):
        # assume the speed will stay constant and evaluate path?

        self.trajectory_set.clear()
        for paths in self.path_set:
            for speeds in self.speed_set:
                trajectory = []
                for i in range(len(paths)):
                    s = paths[i].x
                    l = paths[i].y
                    v = speeds[i]
                    motion_state = MotionState(s, l, v)
                    trajectory.append(motion_state)
                self.add_valid_trajectory(trajectory)
        
        self.generate_trajectory_set_with_dense_info()
        # print("trajectory stop")

    def generate_trajectory_set_with_dense_info(self):
        # may optimize
        for trajectory in self.trajectory_set:
            dense_trajectory = []
            t = 0
            for i in range(len(trajectory) - 1):
                start_motion_state = trajectory[i]
                end_motion_state = trajectory[i + 1]
                l0 = start_motion_state.l
                l1 = end_motion_state.l
                s = end_motion_state.s - start_motion_state.s
                v0 = start_motion_state.v
                v1 = end_motion_state.v 
                #a0 =l0, a1 = 0, 2 * a2 + 3 * a3 * s = 0; a2 + a3 * s = (l1 - l0) / s**2 ==> a3 = -2 * (l1 - l0) / s**3, a2 = 3 * (l1 - l0) / s**2
                s_polynomial = [l0, 0, 3 * (l1 - l0) / s**2, -2 * (l1 - l0) / s**3]

                # assume uniform acceleration
                num_samples = 10
                ds = s / num_samples 
                dv = (v1 - v0) / num_samples
                for j in range(num_samples):
                    s_sample = ds * j
                    l_sample = s_polynomial[0] + s_sample * (s_polynomial[1] + s_sample * (s_polynomial[2] + s_sample * s_polynomial[3]))
                    v_sample = dv * j + v0
                    a_sample = dv
                    t_sample = t

                    v_avg = v_sample + 0.5 * dv
                    if v_avg <= 0.01:
                        v_avg += 0.1
                    dt = ds / v_avg
                    t += dt

                    dense_trajectory.append([s_sample, l_sample, v_sample, a_sample, t_sample])

            self.trajectory_set_with_dense_info.append(dense_trajectory)


    def add_valid_trajectory(self, trajectory):
        self.trajectory_set.append(trajectory)


    """select trajectory"""
    def coordinate_with_other_CAVs(self):
        pass

    def select_best_trajectory_according_to_cost(self):
        obstacles = self.find_surrounding_obstacles_on_road()
        for obstacle in obstacles:
            pass
        pass 

    """improve trajectory"""
    def improve_path(self):
        pass

    def improve_speed(self):
        pass


    """support functions"""



    """******************************************CAV Control Algorithm*****************************************"""
    def find_match_point(self):
        # to do: find the match point of the trajectory reference and use controller to track the speed of it
        idx = 0
        min_distance = 100000
        for i in range(self.num_trajectory_points):
            ego_point = self.point_location()
            point = self.discrete_path_reference[i]
            dis = distance(ego_point, point)
            if dis < min_distance:
                min_distance = dis
                idx = i
        return idx

    def find_look_ahead_angle(self):

        look_ahead_point = self.find_look_ahead_point()
        ego_point = self.point_location()

        look_ahead_vec = Vector(look_ahead_point.x - ego_point.x, look_ahead_point.y - ego_point.y)

        heading_rad = self.state.heading / 180 * np.pi
        heading_vec = Vector(np.cos(heading_rad), np.sin(heading_rad))

        return angle_between_vectors_with_sign(heading_vec, look_ahead_vec)

    def find_look_ahead_point(self):
        look_ahead_distance = self.k * self.state.v
        ego_point = self.point_location()

        look_ahead_point_idx = self.match_point_idx
        min_distance = 10000
        # search the min distance point in the trajectory path points
        for i in range(self.match_point_idx, self.num_trajectory_points):
            point = self.discrete_path_reference[i]
            dis = distance(point, ego_point)
            if np.abs(dis - look_ahead_distance) < min_distance:
                min_distance = np.abs(dis - look_ahead_distance)
                look_ahead_point_idx = i

        self.look_ahead_point = self.discrete_path_reference[look_ahead_point_idx]
        return self.discrete_path_reference[look_ahead_point_idx]


    def longitudinal_lateral_decomposed_control(self):
        self.input.acc = self.longitudinal_control()
        self.input.steer_angle = self.lateral_control()

    def longitudinal_control(self):
        kp = 0.2
        ki = 0.05
        kd = 0.01

        self.pid_dd = (self.pid_dp - (self.v_des - self.state.v)) / self.dt
        self.pid_dp = self.v_des - self.state.v
        self.pid_di += self.pid_dp * self.dt

        # clear the accummulated pi term every 5 seconds
        if self.simulation.count % 500 == 0:
            self.pid_di = 0
        
        # print(f"pid_dp is {self.pid_dp}")
        # print(f"pid_dd is {self.pid_dd}")
        # print(f"pid_di is {self.pid_di}")

        return kp * self.pid_dp + ki * self.pid_di + kd * self.pid_dd
        

    def lateral_control(self):
        # print(f"angle: {self.look_ahead_angle}")
        # print(f"distance: {self.k * self.state.v}")
        # print(f"turn angle: {np.arctan2(2 * self.param.length * self.look_ahead_angle, self.k * self.state.v)}")
        if self.state.v <= 0.001:
            return 0
        return np.arctan2(2 * self.param.length * self.look_ahead_angle, self.k * self.state.v) * 180 / np.pi


    """************************************CAV Simulated Physical Plant **********************************************"""
    
    def update_state(self):
        self.update_lateral_state()
        self.update_longitudinal_state()


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

        beta = self.lateral_state.beta
        heading = self.lateral_state.heading
        d_heading = self.lateral_state.heading_dot
        delta = self.input.steer_angle
        v = self.state.v

        # the dynamics
        if v > 1:
            d_beta = -(cf + cr) / (m * v) * beta + ((cr * lr - cf * lf) / (m * v**2) - 1) * d_heading + cf / (m * v) * delta
            dd_heading = - (cr * lr - cf * lf) / Iz * beta - (cr * lr**2 + cf * lf**2) / (Iz * v) * d_heading + cf * lf / Iz * delta
        else:
            d_beta = 0
            dd_heading = 0
        
        self.lateral_state.beta += self.dt * d_beta
        self.lateral_state.heading += self.dt * d_heading
        self.lateral_state.heading_dot += self.dt * dd_heading

        # update state heading
        self.state.heading = self.lateral_state.heading
        heading = self.lateral_state.heading / 180 * np.pi
        beta = self.lateral_state.beta / 180 * np.pi
        # update state positions
        self.state.x += self.dt * (v * np.cos(heading) - v * beta * np.sin(heading))
        self.state.y += self.dt * (v * np.sin(heading) + v * beta * np.cos(heading))

    def update_longitudinal_state(self):
        ca = 0.01
        cr = 0.001
        acc_des = self.input.acc
        v = self.state.v
        m = 1500

        acc = acc_des - (ca * v**2 + cr * v) / m
        self.state.v += self.dt * acc

