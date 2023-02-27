from .Vehicle import *
from Utilities import *
from collections import deque
import numpy as np
import datetime 
import math

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
        self.is_platooning = 0
        self.prec_veh = None
        self.lateral_state = LateralState(heading = veh_state.heading)

        self.init_computation_module()
        self.init_env_module()
        self.init_frenet_module()
        self.init_target_module()
        self.init_planner_module()
        self.init_control_module()
    
    def add_controller(self, controller):
        self.controller = controller
    
    def add_planner(self, planner):
        self.planner = planner

    def add_target(self, target):
        self.target = target

    # update : target, planner, controller
    def update(self):
        if self.is_platooning == 2:
            # platoon formation is done, switch to platooning control
            self.update_platooning_control()
            return

        self.update_env()
        self.update_target()
        self.update_trajectory_reference()
        self.update_control_command()
        self.update_state()


    def update_platooning_control(self):
        # may develop an acc controller
        # self.input.acc = 0
        # self.steer_angle = 0
        # self.update_state()
        # self.state.heading = 0

        prec_veh = self.find_prec_veh()
        if prec_veh is None:
            self.input.acc = 0.1 * (30 - self.state.v) # 30 is desired speed
        else:
            v_p = prec_veh.state.v - self.state.v
            s_p = prec_veh.state.x - self.state.x - 60
            self.input.acc = 0.58 * v_p + 0.1 * s_p
        self.update_state()
        self.state.heading = 0

    def update_env(self):
        self.update_surrounding_vehicles_on_road()

    def lane_target_has_collision_with_traffic(self, target_point, lane_idx):
        for veh in self.segment.vehicles[lane_idx]:
            if distance(veh.point_location(), target_point) < 8:
                return True
        return False

    # #TODO: Update lane idx after CAV make lane change

    def update_target(self):
        # if it is platooning, stop update its own target
        if self.is_platooning == 1:
            return
        # # target is always in the mid lane
        mid_lane_match_point = Point(self.point_location().x, self.segment.start.y)
        target_location = mid_lane_match_point + self.target_movement_point
        self.target_location = self.point_location() + self.target_movement_point #target_location
        self.target_speed = self.state.v
        self.target_heading = 0
        self.target_time = distance(target_location, self.point_location()) / (self.target_speed + self.state.v) * 2

    def update_trajectory_reference(self):
        if self.simulation.count % 20 == 0:
            start_time = datetime.datetime.now()
            self.generate_trajectory_set()
            # self.select_best_path()
            # self.display_trajectory()
            end_time = datetime.datetime.now()

            self.update_visualization_set()

            self.update_discrete_path_reference()
            self.update_discrete_speed_reference()

            self.motion_plan_compute_time.append((end_time - start_time).microseconds / 1000)
            # print(f"Motion computation time: {(end_time - start_time).microseconds / 1000}")

    #TODO: Update discrete path and speed references
    def update_discrete_path_reference(self):
        new_discrete_path_reference = []
        for location, speed in self.best_trajectory_cartessian:
            new_discrete_path_reference.append(location)
        
        if new_discrete_path_reference:
            self.discrete_path_reference = new_discrete_path_reference


    def update_discrete_speed_reference(self):
        new_discrete_speed_reference = []
        for location, speed in self.best_trajectory_cartessian:
            new_discrete_speed_reference.append(speed)
        
        if new_discrete_speed_reference:
            self.discrete_speed_reference = new_discrete_speed_reference


    def update_control_command(self):
        # print(f"length of {len(self.discrete_path_reference)}")
        
        start_time = datetime.datetime.now()
        if len(self.discrete_path_reference) == 0:
            self.input.acc = 0
            self.input.steer_angle = 0
        else:
            self.controller.update()

        end_time = datetime.datetime.now()
        self.control_compute_time.append((end_time - start_time).microseconds / 1000)
        # print(f"Control computation time: {(end_time - start_time).microseconds / 1000}")



    """******************************************CAV motion trajectory Algorithm*****************************************"""

    def update_surrounding_vehicles_on_road(self):
        self.surrounding_vehicles.clear()
        for segment in self.road.segments:
            for lane_vehicles in segment.vehicles:
                for vehicle in lane_vehicles:
                    ego_point = self.point_location()
                    veh_point = vehicle.point_location()
                    if distance(ego_point, veh_point) <= 100 and vehicle is not self:
                        self.surrounding_vehicles.append(vehicle)
                        # print(f"surround vehicle includes {vehicle.lane}")

    def find_prec_veh(self):
        min_distance = 100000 if self.prec_veh is None else distance(self.point_location(), self.prec_veh.point_location())
        for segment in self.road.segments:
            for lane_vehicles in segment.vehicles:
                for vehicle in lane_vehicles:
                    dis = distance(self.point_location(), vehicle.point_location())
                    if dis < min_distance:
                        self.prec_veh = vehicle
                        min_distance = dis


    def generate_trajectory_set(self):
        self.num_layers = 5
        self.num_path_samples_per_layer = 3 # can only focus on the adjacent lanes
        self.num_speed_samples_per_layer = 5

        self.vmin = 0
        self.vmax = 30

        self.generate_path_set()
        self.select_best_path()

    def select_best_trajectory(self):
        self.coordinate_with_other_CAVs()
        self.best_trajectory = self.select_best_trajectory_according_to_cost()

    def improve_best_trajectory(self):
        # ToDo: iteratively improve path and speed trajectory
        self.improve_path()
        self.improve_speed()


    """trajectory generation"""

    def generate_path_set(self):
        # for straight road
        # node info: (s, l, v, a, k， t)
        #TODO: extend to curve road
        if len(self.discrete_path_reference) > 0:
            start_point = self.discrete_path_reference[self.match_point_idx]
            # print(f"start point is {start_point}")
            s = 0
            l = start_point.y - self.segment.start.y
        else:
            s = 0 
            l = self.point_location().y - self.segment.start.y
        # print(f"s is {s}, l is {l}")
        # print(f"start l is {l}")
        self.path_root = Node(s, l, self.state.v, 0, 0, 0)
        self.path_nodes_vec =  [[] for _ in range(self.num_layers)]
        self.cost = [[[0 for _ in range(self.road.num_lanes)] for _ in range(self.road.num_lanes)] for _ in range(self.num_layers)]
        self.path_nodes = deque([self.path_root])
        self.path_nodes_vec[0].append(self.path_root)

        longitudinal_distance = distance(self.target_location, self.point_location())
        lateral_distance = self.road.num_lanes * self.road.lane_width
        v_distance = self.target_speed - self.state.v
        t_distance = self.target_time - 0

        # sample nodes in the mid
        idx = 0
        for i in range(1, self.num_layers):
            frenet_s = longitudinal_distance / (self.num_layers - 1) * i
            frenet_v = v_distance / (self.num_layers - 1) * i + self.state.v
            frenet_t = t_distance / (self.num_layers - 1) * i

            # num_nodes_in_layer = len(path_nodes) - idx
            cur_len = len(self.path_nodes)
            # print(f"frenet s is {frenet_s}")
            # add next layer nodes into deque
            if i == self.num_layers - 1:
                frenet_l = self.target_location.y - self.segment.start.y
                nex_node = Node(frenet_s, frenet_l, frenet_v, 0, 0, frenet_t)
                self.path_nodes_vec[i].append(nex_node)
                self.path_nodes.append(nex_node)
            else:
                for j in range(self.road.num_lanes):
                    frenet_l = (-self.road.lane_width + j * self.road.lane_width)
                    # print(f"frenet l is {frenet_l}")
                    nex_node = Node(frenet_s, frenet_l, frenet_v, 0, 0, frenet_t)
                    self.path_nodes_vec[i].append(nex_node)
                    self.path_nodes.append(nex_node)


            # construct edges according cur_node and nex_node
            for k in range(idx, cur_len):
                cur_node = self.path_nodes[k]
                # path_nodes.popleft()
                
                for j in range(cur_len, len(self.path_nodes)):
                    nex_node = self.path_nodes[j]
                    
                    collision_priority = self.num_layers - i
                    num_samples = 20
                    collision_priority = i
                    # if i == 0:
                    #     collision_priority = 10
                    #     num_samples = 20
                    # add child
                    cur_node.add_child(nex_node)
                    # add edge
                    edge = Edge(cur_node, nex_node, num_samples)

                    edge.evaluate_edge_cost(self, self.surrounding_vehicles, self.road, collision_priority)
                    cur_node.add_edge(edge)

                    cur_layer_idx = k - idx        
                    nex_layer_idx = j - cur_len
                    self.cost[i-1][cur_layer_idx][nex_layer_idx] = edge.get_cost()

                    # print(f"cur_layer is {i-1}, cur_layer_idx is {cur_layer_idx}, next_layer_idx is {nex_layer_idx}, cost is {edge.get_cost()}")")
            idx = cur_len
        
    def select_best_path(self):
        self.dp = [[0 for _ in range(self.road.num_lanes)] for _ in range(self.num_layers)]
        prev_best_trajectory = self.best_trajectory
        self.best_trajectory.clear()
        self.best_trajectory_cartessian.clear()

        for i in range(1, self.num_layers):
            num_j = self.road.num_lanes if i < self.num_layers - 1 else 1
            for j in range(num_j):
                num_k = self.road.num_lanes if i > 1 else 1
                self.dp[i][j] = min((self.dp[i-1][k] + self.cost[i-1][k][j]) for k in range(num_k))

        # self.print_dp()
        # print(f"best trajectory cost is {self.dp[-1][0]}")
        # self.print_cost(0)

        cur_trajectory_cost = self.dp[-1][0]
        j = 0 # this is the cur_trajectory_cost idx at the final layer
        for i in range(self.num_layers - 1, 0, -1):
            num_k = self.road.num_lanes if i > 1 else 1
            for k in range(num_k):
                if abs(self.dp[i-1][k] + self.cost[i-1][k][j] - cur_trajectory_cost) < 0.001: 
                    # print(f"best node i is {i}, k is {k}")
                    self.best_trajectory.appendleft((self.path_nodes_vec[i-1][k], self.path_nodes_vec[i-1][k].edges[j], (i-1, k)))
                    cur_trajectory_cost -= self.cost[i-1][k][j] # update cost
                    j = k # update j
                    break
    
        for main_node, edge, idx in self.best_trajectory:
            for sample in edge.samples:
                sl_point = Point(sample.s, sample.l)
                cartessian_point = sl_point + Point(self.point_location().x, self.segment.start.y)
                self.best_trajectory_cartessian.append((cartessian_point, sample.v))

        # some times fail because the target point is in the static obstacle
        # assert(cur_trajectory_cost == 0)


    def update_visualization_set(self):
        self.visualization_set.clear()
        self.dense_visualization_set.clear()

        for layer in self.path_nodes_vec:
            for node in layer:
                sample = Point(node.s, node.l)
                point = sample + Point(self.point_location().x, self.segment.start.y)
                self.visualization_set.append(point)

        # traverse the path nodes graph
        path_nodes = deque([self.path_root])
        # idx = 0
        while path_nodes:
            # print(f"node {idx} is visited")

            cur_node = path_nodes[0]
            path_nodes.popleft()

            for edge in cur_node.edges:
                for node in edge.samples:
                    sl_point = Point(node.s, node.l)
                    self.dense_visualization_set.append(sl_point + Point(self.point_location().x, self.segment.start.y))
            
            for child in cur_node.children:
                # print(f"child of node {idx} is visited")
                path_nodes.append(child)

            # idx += 1


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




    """************************************CAV Simulated Physical Plant **********************************************"""
    
    def update_state(self):
        print(f"cav id is {self.permanent_id}; target speed is {self.target_speed}, desired speed is {self.v_des}")
        print(f"cav speed is {self.state.v}, acc is {self.input.acc}, heading is {self.state.heading}, steer is {self.input.steer_angle}")
        print(f"cav location is {self.state.x}, {self.state.y}")
        self.update_kinematic()
        # if self.state.v <= 5:
        #     self.state.v = 0
        # else:
        #     self.update_lateral_state()
        #     self.update_longitudinal_state()
        # print(f"cav state: {self}")

    def update_motion(self):
        pass
    
    def update_kinematic(self):
        # if self.state.v <= 
        phi = self.state.heading / 180 * np.pi
        self.state.x += self.state.v * self.dt * np.cos(phi)
        self.state.y += self.state.v * self.dt * np.sin(phi)
        
        steer_angle = self.input.steer_angle / 180 * np.pi
        self.state.heading += (self.state.v * np.tan(steer_angle) / self.param.length * self.dt) * 180 / np.pi
        self.state.v += self.input.acc * self.dt

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
        if v >= 2:
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

    """************************************Initialization Modules **********************************************"""

    def init_computation_module(self):
        # computation time summary
        self.control_compute_time = []
        self.motion_plan_compute_time = []
        self.target_compute_time = []

    def init_env_module(self):
        # surrounding vehicle module
        self.surrounding_vehicles = []

    def init_target_module(self):
        # target module
        self.target_movement_point = Point(100, 0)
        self.target_location = self.point_location() + self.target_movement_point
        self.target_speed = self.state.v
        self.target_time = distance(self.target_location, self.point_location()) / (self.target_speed + self.state.v) * 2

    def init_planner_module(self):
        # initialize motion planner configuration
        # initialize controller configuration
        # self.discrete_path_reference = []
        # self.discrete_speed_reference = []
        self.control_track_point = None

        self.trajectory_set = []
        self.best_trajectory_idx = deque()
        self.best_trajectory = deque()
        self.path_set = deque()
        self.speed_set = deque()
        self.visualization_set = []
        self.best_trajectory_cartessian = []

        self.path_edges = []
        self.speed_edges = []

        self.best_path = []
        self.best_speed = []

        self.dense_visualization_set = []

        self.trajectory_set_with_dense_info = []

    def init_control_module(self):
        
        # pid configuration
        self.match_point_idx = 0
        self.pid_dp = 0
        self.pid_di = 0
        self.pid_dd = 0

        # pure pursuit configuration
        self.k = 0.5
        self.look_ahead_angle = 0
        self.look_ahead_point = Point(2, 2)

    """Support functions"""

    def print_dp(self):
        for i in range(len(self.dp)):
            print()
            for j in range(len(self.dp[i])):
                print(self.dp[i][j], end=",")
            print()

    def print_cost(self, layer):
        for j in range(self.road.num_lanes):
            print()
            for k in range(self.road.num_lanes):
                print(self.cost[layer][j][k], end=",")
            print()

    def init_frenet_module(self):
        self.reference_line = []
        self.num_trajectory_points = 500
        self.discrete_speed_reference = []
        self.discrete_path_reference = []

        # v = 20
        # for i in range(100):
        #     v += -0.02
        #     self.discrete_speed_reference.append(v)
        # for i in range(100):
        #     v -= 0.00
        #     self.discrete_speed_reference.append(v)
        # for i in range(300):
        #     self.discrete_speed_reference.append(v)

        if self.segment.is_segment_straight_getter():
            start = Point(self.segment.start.x, self.segment.start.y)
            end = Point(self.segment.end.x, self.segment.end.y)
            for i in range(500):
                x = ((500 - i) * start.x + i * end.x) / 500
                y = ((500 - i) * start.y + i * end.y) / 500
                # print(x)
                self.reference_line.append(Point(x, y))
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
                self.reference_line.append(point)























    # """******************************************CAV Control Algorithm*****************************************"""
    # def find_match_point(self):
    #     # TODO: find the match point of the trajectory reference and use controller to track the speed of it
    #     idx = 0
    #     min_distance = 100000
    #     for i in range(len(self.discrete_path_reference)):
    #         ego_point = self.point_location()
    #         point = self.discrete_path_reference[i]
    #         dis = distance(ego_point, point)
    #         if dis < min_distance:
    #             min_distance = dis
    #             idx = i
    #     return idx

    # def find_look_ahead_angle(self):

    #     look_ahead_point = self.find_look_ahead_point()
    #     ego_point = self.point_location()

    #     look_ahead_vec = Vector(look_ahead_point.x - ego_point.x, look_ahead_point.y - ego_point.y)

    #     heading_rad = self.state.heading / 180 * np.pi
    #     heading_vec = Vector(np.cos(heading_rad), np.sin(heading_rad))

    #     return angle_between_vectors_with_sign(heading_vec, look_ahead_vec)

    # def find_look_ahead_point(self):
    #     look_ahead_distance = self.k * self.state.v
    #     ego_point = self.point_location()

    #     look_ahead_point_idx = self.match_point_idx
    #     min_distance = 10000
    #     # search the min distance point in the trajectory path points
    #     for i in range(self.match_point_idx ,len(self.discrete_path_reference)):
    #         point = self.discrete_path_reference[i]
    #         dis = distance(point, ego_point)
    #         if np.abs(dis - look_ahead_distance) < min_distance:
    #             min_distance = np.abs(dis - look_ahead_distance)
    #             look_ahead_point_idx = i

    #     self.look_ahead_point = self.discrete_path_reference[look_ahead_point_idx]
    #     return self.discrete_path_reference[look_ahead_point_idx]


    # def longitudinal_lateral_decomposed_control(self):
    #     self.input.acc = self.longitudinal_control()
    #     self.input.steer_angle = self.lateral_control()

    # def longitudinal_control(self):
    #     kp = 0.2
    #     ki = 0.05
    #     kd = 0.01

    #     self.pid_dd = (self.pid_dp - (self.v_des - self.state.v)) / self.dt
    #     self.pid_dp = self.v_des - self.state.v
    #     self.pid_di += self.pid_dp * self.dt

    #     # clear the accummulated pi term every 5 seconds
    #     if self.simulation.count % 500 == 0:
    #         self.pid_di = 0
        
    #     # print(f"pid_dp is {self.pid_dp}")
    #     # print(f"pid_dd is {self.pid_dd}")
    #     # print(f"pid_di is {self.pid_di}")

    #     return kp * self.pid_dp + ki * self.pid_di + kd * self.pid_dd
        

    # def lateral_control(self):
    #     # print(f"angle: {self.look_ahead_angle}")
    #     # print(f"distance: {self.k * self.state.v}")
    #     # print(f"turn angle: {np.arctan2(2 * self.param.length * self.look_ahead_angle, self.k * self.state.v)}")
    #     if self.state.v <= 0.001:
    #         return 0
    #     # print(f"look ahead angle is {self.look_ahead_angle}, look ahead point is {self.look_ahead_point}")
    #     if math.isnan(self.look_ahead_angle):
    #         return 0
    #     return np.arctan2(2 * self.param.length * self.look_ahead_angle, self.k * self.state.v) * 180 / np.pi
