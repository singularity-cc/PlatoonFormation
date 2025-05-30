# from Vehicle import *
# import sys    
# print("In module Graph sys.path[0], __package__ ==", sys.path[0], __package__)

import sys
sys.path.append(r"C:\Users\zhhyok\OneDrive - Argonne National Laboratory\Desktop\Spring 2023\CooperativeMotionPlanFramework")
from Utilities import *

# from .Polynomial import *
# from .Geometry import *

class Edge:
    def __init__(self, start, end, num_samples = 10):
        self.start = start
        self.end = end

        self.curve = None
        self.samples = [] # Node in it
        self.cost = 0

        self.generate_curve(order = 5)
        self.num_samples = num_samples
        self.generate_edge_samples()

    def get_cost(self):
        # print(f"edge cost is {self.cost}")
        return self.cost

    def generate_edge_samples(self):
        if self.curve is None:
            # print("Edge Curve has not been generated")
            return
        v_s = self.start.v
        v_e = self.end.v
        a_s = self.start.a
        a_e = self.end.a
        t_s = self.start.t
        t_e = self.end.t

        for i in range(self.num_samples):
            s_sample = self.curve.duration() / self.num_samples * i # consider s0 = 0
            l_sample = self.curve.compute(order=0, length=s_sample) #
            v_sample = (v_e * i + (self.num_samples - i) * v_s) / self.num_samples
            a_sample = (a_e * i + (self.num_samples - i) * a_s) / self.num_samples
            k_sample = self.curve.compute(order=1, length=s_sample)
            t_sample = (t_s * i + (self.num_samples - i) * t_e) / self.num_samples
            self.samples.append(Node(s_sample + self.start.s, l_sample, v_sample, a_sample, k_sample, t_sample))
            # print(f"sample node is generated at {s_sample}, {l_sample}")
    
    def evaluate_edge_cost(self, cav, obstacles, road, collision_priority):
        ego_point = cav.point_location()
        collision_cost = self.compute_collision_cost(ego_point, obstacles, road, collision_priority)
        discomfort_cost = self.compute_discomfort_cost()
        lane_change_cost = self.compute_lane_change_cost()
        center_lane_offset_cost = self.compute_center_lane_offset_cost(road)
        target_cost = self.compute_target_cost(cav, road)

        self.cost = collision_cost + center_lane_offset_cost + discomfort_cost + lane_change_cost + target_cost
        return self.cost

    def compute_lane_change_cost(self):
        lane_offset = abs(self.start.l - self.end.l)
        return 5 ** lane_offset

    def compute_collision_cost(self, ego_point, obstacles, road, collision_priority):
        reference_point = Point(ego_point.x, road.segments[0].start.y) # This only works for straight road
        cost = 0
        for sample in self.samples:
            sample_point = reference_point + Point(sample.s, sample.l)
            time = sample.t
            for obstacle in obstacles:
                cur_obs_point = obstacle.point_location()
                # if the obstacle is behind the ego vehicle, do not calculate cost
                if ego_point.x >= cur_obs_point.x + 10:
                    continue
                future_obs_point = cur_obs_point + Point(time * obstacle.state.v, 0)
                if obstacle.category == "CAV":
                    # this is for comparison of CAV no cooperation
                    # cost += self.calculate_collision_cost(sample_point, future_obs_point, collision_priority)

                    cost += self.calculate_collision_cost(sample_point, future_obs_point, collision_priority + 3)
                else:
                    cost += self.calculate_collision_cost(sample_point, future_obs_point, collision_priority)
        return cost

    def compute_discomfort_cost(self):
        cost = 0
        for sample in self.samples:
            a = sample.a
            k = sample.k
            cost += a * a + k * k
        return cost
    
    def compute_target_cost(self, cav, road):
        cost = 0
        for sample in self.samples:
            target_lane_offset = abs(sample.l - (cav.target_location.y - road.segments[0].start.y))
        return 10 * target_lane_offset * target_lane_offset

    def compute_center_lane_offset_cost(self, road):
        cost = 0
        for sample in self.samples:
            distances_to_lane_center = [abs(-road.lane_width - sample.l), abs(-sample.l), abs(road.lane_width - sample.l)]
            min_dis = min(distances_to_lane_center)
            cost += 10 * abs(min_dis * min_dis) 
        return cost

    def compute_path_length_cost(self):
        cost = 0
        for i in range(len(self.samples) - 1):
            point1 = Point(self.samples[i].s, self.samples[i].l)
            point2 = Point(self.samples[i+1].s, self.samples[i+1].l)
            dis = distance(point1, point2)
            cost += 100 * dis

        return cost


    def compute_lane_change_cost(self):
        cost = 0

        return cost

    """Support functions"""
    def calculate_collision_cost(self, sample_point, obs_point, collision_priority):
        if sample_point.x > obs_point.x + 10:
            return 0
        # if sample_point.x < obs_point.x + 40:
        #     return 0

        collision_cost = 0
        dis_min = 3
        if collision_priority == 1:
            M1 = 10000
            w1 = 10
        elif collision_priority == 2:
            M1 = 10000
            w1 = 5
        elif collision_priority == 3:
            M1 = 1000
            w1 = 3
        elif collision_priority == -1:
            M1 = 0
            w1 = 0
        else:
            M1 = 1
            w1 = 1
        
        # will have different cost for longitudinal and lateral collision risks

        lam_static = 1

        radius = 1.5
        ego_circle_centers = [sample_point]
        ego_circle_centers.append(sample_point + Point(radius, 0))
        ego_circle_centers.append(sample_point - Point(radius, 0))
        
        obstacle_circle_centers = [obs_point]
        obstacle_circle_centers.append(obs_point + Point(radius, 0))
        obstacle_circle_centers.append(obs_point - Point(radius, 0))

        for ego_center in ego_circle_centers:
            for obs_center in obstacle_circle_centers:
                dis = distance(ego_center, obs_center)
                longitudinal_dis = abs(ego_center.x - obs_center.x)
                lateral_dis = abs(ego_center.y - obs_center.y)
                if dis <= 2 * radius:
                    collision_cost += M1
                else:
                    collision_cost += w1 * np.exp(- 1 / lam_static * longitudinal_dis)
                    # collision_cost += w1 * np.exp(- 1 / lam_static * dis)
        return collision_cost

    def generate_curve(self, order):
        if order == 5:
            l_s = self.start.l
            s_s = self.start.s
            l_e = self.end.l
            s_e = self.end.s
            k_s = self.start.k
            k_e = self.end.k
            # print(f"Quintic poly start at 0, {l_s}, end at {s_e - s_s}, {l_e}")
            start_motion_state = MotionState1D(l_s, k_s, 0)
            end_motion_state = MotionState1D(l_e, k_e, 0)
            self.curve = QuinticPolynomial(start_motion_state, end_motion_state, s_e - s_s)
        elif order == 4:
            print("Should generate quartic poly")
        


        
        

class Node:
    def __init__(self, s, l, v, a, k, t):
        self.node = (s, l, v, a, k, t)
        self.s = s
        self.l = l
        self.v = v
        self.a = a
        self.k = k
        self.t = t
        self.edges = []
        self.children = []


    def add_edge(self, edge): #node : (s, l, v, a, k)
        self.edges.append(edge)


    def add_child(self, node):
        self.children.append(node)



if __name__ == "__main__":
    obstacle = Point(33, 4)
    
    start_node = Node(0, 4, 0, 0, 0)
    end_node = Node(37.5, -4, 0, 0, 0)
    edge = Edge(start_node, end_node)
    print(f"cost is {edge.compute_collision_cost(obstacle)}")

    end_node = Node(37.5, 0, 0, 0, 0)
    edge = Edge(start_node, end_node)
    print(f"cost is {edge.compute_collision_cost(obstacle)}")

    end_node = Node(37.5, 4, 0, 0, 0)
    edge = Edge(start_node, end_node)
    print(f"cost is {edge.compute_collision_cost(obstacle)}")
