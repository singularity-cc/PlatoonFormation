
from MotionPlanner import *
from Vehicle import *
from Utilities import *
from collections import deque
import numpy as np
import datetime
import math


class LatticePlanner:
    def __init__(self, cav):
        self.cav = cav
        self.init_planner_module()
        self.init_frenet_module()

    def init_planner_module(self):
        # initialize motion planner configuration
        self.cav.control_track_point = None

        self.cav.best_trajectory = deque()

        # cav path and speed sets
        self.cav.path_set = deque()
        self.cav.speed_set = deque()
        self.cav.trajectory_set = []

        # cav planner visualization
        self.cav.visualization_set = []
        self.cav.best_trajectory_cartessian = []
        self.cav.dense_visualization_set = []
        self.cav.trajectory_set_with_dense_info = []

        # init parameters
        self.cav.num_layers = 5
        self.cav.num_path_nodes_per_layer = 3  # can only focus on the adjacent lanes
        self.cav.num_speed_nodes_per_layer = 0
        # TODO: refactor + 1, the + 1 is because of the extra speed sample from path node
        self.cav.num_trajectory_nodes_per_layer = (self.cav.num_speed_nodes_per_layer + 1) * \
            self.cav.num_path_nodes_per_layer
        self.num_evaluation_samples_per_trajectory = 10
        self.cav.vmin = 15
        self.cav.vmax = 30

    def init_frenet_module(self):
        self.cav.reference_line = []
        self.cav.num_trajectory_points = 500
        self.cav.discrete_speed_reference = []
        self.cav.discrete_path_reference = []

        # init reference line to be along the road segment lane center
        if self.cav.segment.is_segment_straight_getter():
            start = Point(self.cav.segment.start.x, self.cav.segment.start.y)
            end = Point(self.cav.segment.end.x, self.cav.segment.end.y)
            for i in range(500):
                x = ((500 - i) * start.x + i * end.x) / 500
                y = ((500 - i) * start.y + i * end.y) / 500
                # print(x)
                self.cav.reference_line.append(Point(x, y))
        # if segment is not straight
        else:
            arc_start_angle = self.cav.segment.arc_start_angle
            arc_end_angle = self.cav.segment.arc_end_angle
            arc_center = self.cav.segment.arc_center
            arc_radius = self.cav.segment.arc_radius

            for i in range(500):
                angle = ((500 - i) * arc_start_angle +
                         i * arc_end_angle) / 5000
                rad = angle * np.pi / 180.0
                angle_vec = Vector(np.cos(rad), np.sin(rad))
                center_to_point_vec = angle_vec * arc_radius
                point = Point(arc_center.x + center_to_point_vec.x,
                              arc_center.y + center_to_point_vec.y)
                self.cav.reference_line.append(point)

    def update(self):
        if self.cav.simulation.count % 20 == 0:
            start_time = datetime.datetime.now()
            self.generate_trajectory_set()
            self.select_best_trajectory()
            # self.display_trajectory()
            end_time = datetime.datetime.now()

            self.update_visualization_set()

            # update path and speed reference for controller module
            self.update_discrete_path_reference()
            self.update_discrete_speed_reference()

            self.cav.motion_plan_compute_time.append(
                (end_time - start_time).microseconds / 1000)
            print(
                f"Motion computation time: {(end_time - start_time).microseconds / 1000}")
        pass

    def generate_trajectory_set(self):
        self.generate_path_set()
        self.generate_speed_set()
        self.combine_path_speed_set()

    # ######################################################################

    # #TODO: Update discrete path and speed references
    def update_discrete_path_reference(self):
        new_discrete_path_reference = []
        for location, speed in self.cav.best_trajectory_cartessian:
            new_discrete_path_reference.append(location)

        if new_discrete_path_reference:
            self.cav.discrete_path_reference = new_discrete_path_reference

    def update_discrete_speed_reference(self):
        new_discrete_speed_reference = []
        for location, speed in self.cav.best_trajectory_cartessian:
            new_discrete_speed_reference.append(speed)

        if new_discrete_speed_reference:
            self.cav.discrete_speed_reference = new_discrete_speed_reference

    #######################################################################

    # """trajectory generation"""

    def generate_path_set(self):
        # for straight road
        # TODO: extend to curve road
        if len(self.cav.discrete_path_reference) > 0:
            start_point = self.cav.discrete_path_reference[self.cav.match_point_idx]
            # print(f"start point is {start_point}")
            s = 0
            l = start_point.y - self.cav.segment.start.y
        else:
            s = 0
            l = self.cav.point_location().y - self.cav.segment.start.y
        # print(f"s is {s}, l is {l}")
        # print(f"start l is {l}")

        # initialize trajectory nodes
        self.cav.path_root = Node(s, l, self.cav.state.v, 0, 0, 0)
        self.cav.path_nodes_vec = [[] for _ in range(self.cav.num_layers)]
        self.cav.path_nodes = deque([self.cav.path_root])
        self.cav.path_nodes_vec[0].append(self.cav.path_root)

        # initialize longitudinal distance and v, t distances
        longitudinal_distance = min(
            distance(self.cav.target_location, self.cav.point_location()), 100)
        # lateral_distance = self.cav.road.num_lanes * self.cav.road.lane_width
        v_distance = self.cav.target_speed - self.cav.state.v
        t_distance = self.cav.target_time - 0

        # sample nodes in the mid
        idx = 0
        for i in range(1, self.cav.num_layers):
            frenet_s = longitudinal_distance / (self.cav.num_layers - 1) * i
            frenet_v = v_distance / \
                (self.cav.num_layers - 1) * i + self.cav.state.v
            frenet_t = t_distance / (self.cav.num_layers - 1) * i
            # add next layer nodes into deque
            if i == self.cav.num_layers - 1:
                frenet_l = self.cav.target_location.y - self.cav.segment.start.y
                nex_node = Node(frenet_s, frenet_l, frenet_v, 0, 0, frenet_t)
                self.cav.path_nodes_vec[i].append(nex_node)
                self.cav.path_nodes.append(nex_node)
            else:
                for j in range(self.cav.num_path_nodes_per_layer):
                    frenet_l = (-self.cav.road.lane_width +
                                j * self.cav.road.lane_width)
                    # print(f"frenet l is {frenet_l}")
                    nex_node = Node(frenet_s, frenet_l,
                                    frenet_v, 0, 0, frenet_t)

                    self.cav.path_nodes_vec[i].append(nex_node)
                    self.cav.path_nodes.append(nex_node)

    def generate_speed_set(self):
        self.cav.speed_nodes_vec = [[] for _ in range(self.cav.num_layers)]

        # add initial speed
        # self.cav.speed_nodes_vec[0].append(self.cav.state.v)

        # add middle layer speeds
        for i in range(1, self.cav.num_layers - 1):
            for j in range(self.cav.num_speed_nodes_per_layer):
                v = self.cav.vmin + \
                    (self.cav.vmax - self.cav.vmin) / \
                    self.cav.num_speed_nodes_per_layer * j
                self.cav.speed_nodes_vec[i].append(v)

        # add target speed
        # self.cav.speed_nodes_vec[-1].append(self.cav.target_speed)

    def combine_path_speed_set(self):
        self.cav.trajectory_nodes_vec = [[]
                                         for _ in range(self.cav.num_layers)]

        self.cav.cost = [[[0 for _ in range(self.cav.num_trajectory_nodes_per_layer)] for _ in range(
            self.cav.num_trajectory_nodes_per_layer)] for _ in range(self.cav.num_layers)]

        # generate trajectory nodes

        for i in range(0, self.cav.num_layers):
            speed_nodes = self.cav.speed_nodes_vec[i]
            path_nodes = self.cav.path_nodes_vec[i]

            for path_node in path_nodes:
                # add path node with speed profiles to reach target speed
                # TODO: refactor to use speed set
                self.cav.trajectory_nodes_vec[i].append(path_node)
                for speed_node in speed_nodes:
                    s = path_node.s
                    l = path_node.l
                    v = speed_node
                    self.cav.trajectory_nodes_vec[i].append(
                        Node(s, l, v, 0, 0, 0))

        # generate trajectory edges and costs
        for i in range(1, self.cav.num_layers):
            cur_layer_trajectory_nodes = self.cav.trajectory_nodes_vec[i - 1]
            nex_layer_trajectory_nodes = self.cav.trajectory_nodes_vec[i]
            for j in range(len(cur_layer_trajectory_nodes)):
                cur_trajectory_node = cur_layer_trajectory_nodes[j]
                for k in range(len(nex_layer_trajectory_nodes)):
                    nex_trajectory_node = nex_layer_trajectory_nodes[k]

                    collision_priority = i

                    cur_trajectory_node.add_child(nex_trajectory_node)
                    trajectory_edge = Edge(
                        cur_trajectory_node, nex_trajectory_node, self.num_evaluation_samples_per_trajectory)
                    trajectory_edge.evaluate_edge_cost(
                        self.cav, self.cav.surrounding_vehicles, self.cav.road, collision_priority)
                    cur_trajectory_node.add_edge(trajectory_edge)

                    self.cav.cost[i-1][j][k] = trajectory_edge.get_cost()

    def select_best_trajectory(self):

        self.cav.dp = [[0 for _ in range(self.cav.num_trajectory_nodes_per_layer)]
                       for _ in range(self.cav.num_layers)]
        prev_best_trajectory = self.cav.best_trajectory
        self.cav.best_trajectory.clear()
        self.cav.best_trajectory_cartessian.clear()

        for i in range(1, self.cav.num_layers):
            num_j = self.cav.num_trajectory_nodes_per_layer if i < self.cav.num_layers - \
                1 else 1  # self.cav.num_speed_nodes_per_layer, assuming the last layer only have one speed node
            for j in range(num_j):
                num_k = self.cav.num_trajectory_nodes_per_layer if i > 1 else 1
                self.cav.dp[i][j] = min(
                    (self.cav.dp[i-1][k] + self.cav.cost[i-1][k][j]) for k in range(num_k))

        # self.print_dp()
        # print(f"best trajectory cost is {self.dp[-1][0]}")
        # self.print_cost(0)

        cur_trajectory_cost = self.cav.dp[-1][0]
        j = 0  # this is the cur_trajectory_cost idx at the final layer
        for i in range(self.cav.num_layers - 1, 0, -1):
            num_k = self.cav.num_trajectory_nodes_per_layer if i > 1 else 1
            for k in range(num_k):
                if abs(self.cav.dp[i-1][k] + self.cav.cost[i-1][k][j] - cur_trajectory_cost) < 0.001:
                    # print(f"best node i is {i}, k is {k}")
                    self.cav.best_trajectory.appendleft(
                        (self.cav.trajectory_nodes_vec[i-1][k], self.cav.trajectory_nodes_vec[i-1][k].edges[j], (i-1, k)))
                    # update cost
                    cur_trajectory_cost -= self.cav.cost[i-1][k][j]
                    j = k  # update j
                    break

        for main_node, edge, idx in self.cav.best_trajectory:
            for sample in edge.samples:
                sl_point = Point(sample.s, sample.l)
                cartessian_point = sl_point + \
                    Point(self.cav.point_location().x,
                          self.cav.segment.start.y)
                self.cav.best_trajectory_cartessian.append(
                    (cartessian_point, sample.v))

        # some times fail because the target point is in the static obstacle
        # assert(cur_trajectory_cost == 0)

    def update_visualization_set(self):
        self.cav.visualization_set.clear()
        self.cav.dense_visualization_set.clear()

        for layer in self.cav.path_nodes_vec:
            for node in layer:
                sample = Point(node.s, node.l)
                point = sample + \
                    Point(self.cav.point_location().x,
                          self.cav.segment.start.y)
                self.cav.visualization_set.append(point)

        # traverse the path nodes graph
        path_nodes = deque([self.cav.path_root])
        # idx = 0
        while path_nodes:
            # print(f"node {idx} is visited")

            cur_node = path_nodes[0]
            path_nodes.popleft()

            for edge in cur_node.edges:
                for node in edge.samples:
                    sl_point = Point(node.s, node.l)
                    self.cav.dense_visualization_set.append(
                        sl_point + Point(self.cav.point_location().x, self.cav.segment.start.y))

            for child in cur_node.children:
                # print(f"child of node {idx} is visited")
                path_nodes.append(child)

            # idx += 1

    """improve trajectory"""

    def coordinate_with_other_CAVs(self):
        # TODO(HANYU): add cooperation algorithm
        pass

    def improve_best_trajectory(self):
        # TODO(HANYU): iteratively improve path and speed trajectory
        self.improve_path()
        self.improve_speed()

    """improve trajectory"""

    def improve_path(self):
        pass

    def improve_speed(self):
        pass

    def find_prec_veh(self):
        # if self.prec_veh is None else distance(self.point_location(), self.prec_veh.point_location())
        min_distance = 100000
        for segment in self.cav.road.segments:
            for lane_vehicles in segment.vehicles:
                for vehicle in lane_vehicles:
                    if abs(vehicle.state.y - self.state.y) >= self.road.lane_width / 2:
                        continue
                    if vehicle is self:
                        continue
                    # distance(self.point_location(), vehicle.point_location())
                    dis = vehicle.state.x - self.cav.state.x
                    if dis > 0 and dis < min_distance:
                        self.cav.prec_veh = vehicle
                        min_distance = dis

        return self.cav.prec_veh
