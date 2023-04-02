from .Vehicle import *
from Vehicle import *
from Utilities import *
from Platoon import *
from MotionPlanner import *
from Controller import *
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
        self.lateral_state = LateralState(heading = veh_state.heading)

        self.init_computation_module()
        self.init_env_module()
        self.init_target_module()
    
    def add_controller(self, controller):
        self.controller = controller
    
    def add_planner(self, planner):
        self.planner = planner

    def add_decider(self, decider):
        self.decider = decider

    def add_plant(self, plant):
        self.plant = plant

    # update : target, planner, controller
    def update(self):
        # print(f"cav id is {self.permanent_id}; target speed is {self.target_speed}, desired speed is {self.v_des}")
        # print(f"cav speed is {self.state.v}, acc is {self.input.acc}, heading is {self.state.heading}, steer is {self.input.steer_angle}")
        # print(f"cav location is {self.state.x}, {self.state.y}")
        # print(f"cav lane is {self.lane}")
        # doing platooning control or not is also controlled by decider
        if self.is_platooning == 2:
            # platoon formation is done, switch to platooning control
            self.update_platooning_control()
            return
        
        # update environment
        self.update_env()
        # update CAV target state
        self.update_target()
        # update CAV trajectory reference
        self.update_trajectory_reference()
        # update CAV control command
        self.update_control_command()
        # update CAV physical state
        self.update_state()
        if self.simulation.count % 100 == 0:
            print(f"cav lane is {self.lane}")
            self.update_labels()

    def update_platooning_control(self):
        prec_veh = None
        prec_veh = self.find_prec_veh()
        # print(prec_veh)
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
        
        # self.decider.update()

        # if it is platooning, stop update its own target
        if self.is_platooning == 1:
            self.decider.update()
            return

        # # target is always in the mid lane if not in platooning formation
        mid_lane_match_point = Point(self.point_location().x, self.segment.start.y)
        target_location = mid_lane_match_point + self.target_movement_point
        self.target_location = self.point_location() + self.target_movement_point 
        #target_location # self.point_location() + self.target_movement_point #target_location
        self.target_speed = self.state.v
        self.target_heading = 0
        self.target_time = distance(target_location, self.point_location()) / (self.target_speed + self.state.v) * 2
        

    def update_trajectory_reference(self):
        self.planner.update()


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


    def update_state(self):
        self.plant.update()

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
        self.target_movement_point = Point(300, 0)
        self.target_location = self.point_location() + self.target_movement_point
        self.target_speed = self.state.v
        self.target_time = distance(self.target_location, self.point_location()) / (self.target_speed + self.state.v) * 2

    # """Support functions"""

    # # def print_dp(self):
    # #     for i in range(len(self.dp)):
    # #         print()
    # #         for j in range(len(self.dp[i])):
    # #             print(self.dp[i][j], end=",")
    # #         print()

    # # def print_cost(self, layer):
    # #     for j in range(self.road.num_lanes):
    # #         print()
    # #         for k in range(self.road.num_lanes):
    # #             print(self.cost[layer][j][k], end=",")
    # #         print()
