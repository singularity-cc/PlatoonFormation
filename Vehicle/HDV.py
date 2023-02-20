import numpy as np
from Road import *
from Utilities import *
from .Vehicle import *
import random


class HDV(Vehicle):
    """Human drive vehicle class"""


    def __init__(self, dt, simulation, v_des, permanent_id, lane, id, veh_state, veh_param, veh_input = VehicleInput(0, 0)):
        # we need some label to index the HDV lane, id....
        super().__init__(dt, simulation, v_des, permanent_id, lane, id, veh_state, veh_param, veh_input)
    
        self.category = "HDV"
        self.energy_consumption = 0
        # initialize lane change variables
        self.init_lane_change_variables()

    def init_lane_change_variables(self):
        """Initialize lane change data members"""
        self.is_in_lane_change = False
        self.lane_change_duration = None
        self.lane_change_turn_left = None
        self.lane_idx_change = 0
        self.lane_change_timer = None
        self.lane_change_target_state = None
        self.lane_change_dx = None
        self.lane_change_dy = None
        self.lane_change_dheading = None



    """******************************************  Interface: First Level Main Functions  **********************************************************"""
    # update vehicle information
    def update(self):
        """Interface Function to update HDV decisions and states"""
        # self.lane_change_decision()
        self.car_following_decision()
        self.update_state()
        self.update_consumption()
        # self.record_state()

    def update_state(self):
        """Update the HDV states including speed, position, lane change state, segment and road"""
        self.update_speed()
        self.update_position()
        self.update_state_after_lane_change()
        self.update_segment()
        self.update_road()

    def car_following_decision(self):
        """If vehicle is in lane-change state"""
        if self.is_in_lane_change:
            self.input.acc = 0
            return 

        """If vehicle is not lane-change, use IDM car-following model"""
        self.input.acc = self.IDM_car_following()

    def lane_change_decision(self):
        """No additional lane change if the vehicle is doing lane change"""
        if self.is_in_lane_change:
            return

        """Do lane change decision using random algorithm"""
        self.MOBIL_lane_change()
        # self.random_lane_change()

    def update_consumption(self):
        self.energy_consumption += self.energy_consumption_model()
        print(f"vehicle energy consumption: {self.energy_consumption}")

    """****************************************** Second Level Main Functions  **********************************************************"""
    # def record_state(self):
    #     self.prev_state = self.state

    def update_segment(self):
        """Todo: Update the HDV road segment information"""
        segment_end_point = self.segment.end.convert_to_point()
        vehicle_current_point = self.point_location()
        # vehicle_previous_point = Point(self.prev_state.x, self.prev_state.y)
        # line_segment = LineSegment(vehicle_current_point, vehicle_previous_point)

        if distance(vehicle_current_point, segment_end_point) < self.road.lane_width * self.road.num_lanes:
            self.segment.remove_vehicle(self)
            self.segment.reorganize_vehicles_in_lane(self.lane)

        # if line_segment.is_point_projection_in(segment_end_point):
        #     self.segment.remove_vehicle(self)
    
    def update_road(self):
        """Todo: Update the HDV road information"""
        pass

    def update_state_after_lane_change(self):
        """Update the HDV lane-change state and lane index and id after lane change"""
        if self.is_in_lane_change and self.lane_change_timer >= self.lane_change_duration:
            self.is_in_lane_change = False # lane change finishes
            self.update_lane_and_id()

    def update_speed(self):
        """Update the HDV speed information"""
        self.state.v += self.dt * self.input.acc   

    def update_position(self):
        prev_pos = Point(self.state.x, self.state.y)
        """Update the HDV position information"""
        if self.is_in_lane_change:
            self.update_position_in_lane_change()
        else:
            self.update_position_in_car_following()
        cur_pos = Point(self.state.x, self.state.y)

        self.travel_distance += distance(prev_pos, cur_pos)


    def IDM_car_following(self):
        """Intelligent Driving Model to simulate HDV car-following maneuvers"""
        v = self.state.v
        T = 1.5
        a = 0.73
        b = 1.67
        delta = 4
        l = self.param.length
        s0 = 2 + l # buffer distance will count vehicle length
        x = self.state.x
        y = self.state.y
        # print(f"id is: {self.id}")
        if self.id == 0:
            s = 100000 # s is net distance: x_a-1 - x_a - l_a-1: if there is no preceding vehicle
            dv = 0.0 # dv is speed difference between preceding vehicle and itself: dv = v_a - v_a-1
        else:
            precede_veh = self.segment.vehicles[self.lane][self.id - 1]
            s = np.sqrt((precede_veh.state.x - x)**2 + (precede_veh.state.y - y)**2) - l # why 1 is too small
            dv = v - precede_veh.state.v
        s_des = s0 + v * T + v * dv / (2 * np.sqrt(a * b))

        # if s < l + T * v - v ** 2 / self.acc_min:
        #     acc = self.acc_min
        #     acc = max(acc, (self.v_min - v) / self.dt)
            # if s < l + 5:
            #     acc = -7
        # IDM equation
        if v < 0:
            acc = self.acc_max
        else:
            acc = a * (1 - (v / self.v_des) ** delta - (s_des / s) ** 2)
            acc = min(acc, self.acc_max)
            acc = max(acc, self.acc_min)
            acc = min(acc, (self.v_max - v) / self.dt)
            acc = max(acc, (self.v_min - v) / self.dt)
        return acc

    def IDM_predict(self, precede_veh):
        # if precede_veh is None:
        #     s = 100000

        v = self.state.v
        T = 1.5
        a = 0.73
        b = 1.67
        delta = 4
        l = self.param.length
        s0 = 2 + l # buffer distance will count vehicle length
        x = self.state.x
        y = self.state.y
        s = np.sqrt((precede_veh.state.x - x)**2 + (precede_veh.state.y - y)**2) - l if precede_veh is not None else 100000 # why 1 is too small
        dv = v - precede_veh.state.v if precede_veh is not None else 0.0
        s_des = s0 + v * T + v * dv / (2 * np.sqrt(a * b))
        acc = a * (1 - (v / self.v_des) ** delta - (s_des / s) ** 2)  
        return acc     

    def MOBIL_lane_change(self):
        """MOBIL lane change algorithm to do lane change decisions by calculating the incentives"""
        precede_veh = self.segment.vehicles[self.lane][self.id - 1] if self.id >= 1 else None
        follow_veh = self.segment.vehicles[self.lane][self.id + 1] if self.id + 1 < len(self.segment.vehicles[self.lane]) else None

        left_precede_veh, left_follow_veh = self.surround_vehicles_in_lane(self.lane + 1) if self.segment.is_clockwise else self.surround_vehicles_in_lane(self.lane - 1)
        right_precede_veh, right_follow_veh = self.surround_vehicles_in_lane(self.lane - 1) if self.segment.is_clockwise else self.surround_vehicles_in_lane(self.lane + 1)

        left_lane_change_benefit = -1
        right_lane_change_benefit = -1

        ego_a_no_lane_change = self.IDM_predict(precede_veh)
        fol_a1_no_lane_change = follow_veh.IDM_predict(self) if follow_veh is not None else 0

        if self.is_gap_acceptable(left_precede_veh, left_follow_veh) and left_precede_veh != -1:
            ego_a_after_lane_change = self.IDM_predict(left_precede_veh)
            fol_a1_after_lane_change = follow_veh.IDM_predict(precede_veh) if follow_veh is not None else 0
            fol_a2_no_lane_change = left_follow_veh.IDM_predict(left_precede_veh)
            fol_a2_after_lane_change = left_follow_veh.IDM_predict(self)
            left_lane_change_benefit = ego_a_after_lane_change + fol_a1_after_lane_change + fol_a2_after_lane_change - ego_a_no_lane_change - fol_a1_no_lane_change - fol_a2_no_lane_change
        if self.is_gap_acceptable(right_precede_veh, right_follow_veh) and right_precede_veh != -1:
            ego_a_after_lane_change = self.IDM_predict(right_precede_veh)
            fol_a1_after_lane_change = follow_veh.IDM_predict(precede_veh) if follow_veh is not None else 0
            fol_a2_no_lane_change = right_follow_veh.IDM_predict(right_precede_veh)
            fol_a2_after_lane_change = right_follow_veh.IDM_predict(self)
            right_lane_change_benefit = ego_a_after_lane_change + fol_a1_after_lane_change + fol_a2_after_lane_change - ego_a_no_lane_change - fol_a1_no_lane_change - fol_a2_no_lane_change

        if left_lane_change_benefit > right_lane_change_benefit > 0:
            self.init_lane_change(is_left = True)
        elif right_lane_change_benefit > left_lane_change_benefit > 0:
            self.init_lane_change(is_left = False)            

        pass

    def random_lane_change(self):
        """Random Lane change algorithm to do lane change decisions randomly once the gap is acceptable"""
        precede_veh = self.segment.vehicles[self.lane][self.id - 1] if self.id >= 1 else None
        follow_veh = self.segment.vehicles[self.lane][self.id + 1] if self.id + 1 < len(self.segment.vehicles[self.lane]) else None

        left_precede_veh, left_follow_veh = self.surround_vehicles_in_lane(self.lane + 1) if self.segment.is_clockwise else self.surround_vehicles_in_lane(self.lane - 1)
        right_precede_veh, right_follow_veh = self.surround_vehicles_in_lane(self.lane - 1) if self.segment.is_clockwise else self.surround_vehicles_in_lane(self.lane + 1)

        if self.is_gap_acceptable(left_precede_veh, left_follow_veh) and random.uniform(0, 1) < 0.2: #self.lane >= 1 and
            self.init_lane_change(is_left =True)

        elif self.is_gap_acceptable(right_precede_veh, right_follow_veh) and random.uniform(0, 1) < 0.5: #self.lane < self.segment.num_lanes - 1 and 
            self.init_lane_change(is_left = False)

    def energy_consumption_model(self):
        u = self.input.acc
        v = self.state.v
        instant_c = -753.7 + 9.7326 * v - 0.3014 * v**2 + 0.0053 * v**3 + 44.3809 * u + 5.1753 * u * v - 0.0742 * u * v**2 + 0.0006 * u * v**3 \
        + 17.1641 * u**2 + 0.2942 * u**2 * v + 0.0109 * u**2 * v**2 - 0.001 * u**2 * v**3 \
        -4.2024 * u**3 - 0.7068 * u**3 * v + 0.0116 * u**3 * v**2 - 0.0006 * u**3 * v**3
        print(np.exp(0.01 * instant_c))
        return np.exp(0.01 * instant_c)


    """****************************************** Third Level Main Functions  **********************************************************"""

    def update_lane_and_id(self):
        """Update the HDV lane and id after lane change"""

        self.segment.update_vehicle_lane(self, self.lane_change_turn_left)

    def update_position_in_lane_change(self):
        """Update the HDV position during lane change"""
        self.do_lane_change()
        self.lane_change_timer += self.dt

    def update_position_in_car_following(self):
        """Update the HDV position during car following"""
        d = self.state.v * self.dt
        if self.segment.is_segment_straight_getter():
            """if current road segment is straight"""
            vec_direction = Vector(self.segment.end.x - self.segment.start.x, self.segment.end.y - self.segment.start.y)
            vec_direction.normalize()
            self.state.x += d * vec_direction.x
            self.state.y += d * vec_direction.y
        else:
            """if current road segment is an arc"""
            lane_width = self.road.lane_width
            num_lanes = self.road.num_lanes
            lane_factor = self.lane - (num_lanes // 2)
            radius = self.segment.arc_radius + lane_width * lane_factor
            theta = d / radius
            vec_direction = Vector(np.cos(self.state.heading / 180.0 * np.pi), np.sin(self.state.heading / 180.0 * np.pi))
            vec_direction.normalize()
            self.state.x += d * vec_direction.x
            self.state.y += d * vec_direction.y
            self.state.heading += theta / np.pi * 180.0 # clockwise or anticlockwise

    def init_lane_change(self, is_left):
        """# initilaize the lane change maneuver: set up lane change direction (left or right lane change), lane change duration, target state after lane change, etc."""
        if self.is_in_lane_change == False:
            self.lane_change_turn_left = is_left
            self.lane_idx_change = -1 if is_left else 1
            self.is_in_lane_change = True
            self.lane_change_duration = random.uniform(0.3, 1.5)
            self.lane_change_timer = 0
            self.find_lane_change_target_state()
            self.calculate_lane_change_state_change()

    def do_lane_change(self):
        """# do lane change maneuver: update vehicle states during the lane change"""
        self.state.x += self.lane_change_dx
        self.state.y += self.lane_change_dy
        self.state.heading += self.lane_change_dheading 

    
    """****************************************** Support Functions  **********************************************************"""

    def find_lane_change_target_state_in_straight_road(self):
        """ find target state after lane change in straight road """
        d = self.state.v * self.lane_change_duration
        theta = np.arctan(self.road.lane_width / d)
        vec_direction = Vector(np.cos(self.state.heading / 180.0 * np.pi), np.sin(self.state.heading / 180.0 * np.pi))
        vec_direction.normalize()
        if self.lane_change_turn_left:
            vec_direction.clockwise_rotate(-theta)
        else:
            vec_direction.anti_clockwise_rotate(-theta)
        self.lane_change_target_state =  VehicleState(self.state.x + d * vec_direction.x, self.state.y + d * vec_direction.y, self.state.v, self.state.heading)

    def find_lane_change_target_state_in_arc_road(self):
        """ find target state after lane change in arc road """
        d = self.state.v * self.lane_change_duration
        theta = d / self.segment.arc_radius
        center_to_veh_vector = Vector(self.state.x - self.segment.arc_center.x, self.state.y - self.segment.arc_center.y)
        center_to_veh_vector.normalize()
        lane_width = self.road.lane_width
        num_lanes = self.road.num_lanes
        lane_factor = self.lane - (num_lanes // 2)
        radius = self.segment.arc_radius + lane_width * lane_factor

        if self.lane_change_turn_left:
            if self.segment.is_clockwise:
                print(center_to_veh_vector)
                center_to_veh_vector.anti_clockwise_rotate(theta)
                radius += lane_width
                target = VehicleState(self.segment.arc_center.x + radius * center_to_veh_vector.x, \
                self.segment.arc_center.y + radius * center_to_veh_vector.y, self.state.v, self.state.heading + theta / np.pi * 180.0)
            else:
                center_to_veh_vector.clockwise_rotate(theta)
                radius -= lane_width
                target = VehicleState(self.segment.arc_center.x + radius * center_to_veh_vector.x, \
                self.segment.arc_center.y + radius * center_to_veh_vector.y, self.state.v, self.state.heading - theta / np.pi * 180.0)
        else:
            if self.segment.is_clockwise:
                center_to_veh_vector.anti_clockwise_rotate(theta)
                radius -= lane_width
                target = VehicleState(self.segment.arc_center.x + radius * center_to_veh_vector.x, \
                self.segment.arc_center.y + radius * center_to_veh_vector.y, self.state.v, self.state.heading + theta / np.pi * 180.0)
            else:
                center_to_veh_vector.clockwise_rotate(theta)
                radius += lane_width
                target = VehicleState(self.segment.arc_center.x + radius * center_to_veh_vector.x, \
                self.segment.arc_center.y + radius * center_to_veh_vector.y, self.state.v, self.state.heading - theta / np.pi * 180.0)
        
        self.lane_change_target_state = target       

    def find_lane_change_target_state(self):
        """find the target state after lane change, two scnearios: (1) lane change in straight road, (2) lane change in arc road """
        if self.segment.is_segment_straight_getter():
            self.find_lane_change_target_state_in_straight_road()
        else:
            self.find_lane_change_target_state_in_arc_road()

    def calculate_lane_change_state_change(self):
        """ calculate the lane change state change during one simulation time interval (self.dt) """
        time_intervals = self.lane_change_duration / self.dt
        self.lane_change_dx = (self.lane_change_target_state.x - self.state.x) / time_intervals
        self.lane_change_dy = (self.lane_change_target_state.y - self.state.y) / time_intervals
        self.lane_change_dheading = (self.lane_change_target_state.heading - self.state.heading) / time_intervals



    """******************************************  Unitlity Functions  **********************************************************"""

    def distance_to_vehicle(self, veh):
        """check Euclidean distance between two vehicles"""
        return np.sqrt((veh.state.x - self.state.x)**2 + (veh.state.y - self.state.y)**2)

    def is_gap_acceptable(self, prec_veh, fol_veh):
        """check if the lane change gap is acceptable"""
        if prec_veh == -1 or fol_veh == -1:
            return False

        T = 1.5
        condition1 = self.distance_to_vehicle(prec_veh) >= self.param.length + T * self.state.v if prec_veh is not None else True
        condition2 = self.distance_to_vehicle(fol_veh) >= self.param.length +  T * fol_veh.state.v if fol_veh is not None else True

        return condition1 and condition2

    
    def surround_vehicles_in_lane(self, lane_idx):
        """obtain ego vehicle's surrounding vehicles in another lane to get prepared for lane change"""
        if lane_idx < 0 or lane_idx > self.segment.num_lanes - 1:
            return -1, -1

        vehicles = self.segment.vehicles[lane_idx]
        for i in range(len(vehicles) - 1):
            prec_veh = vehicles[i]
            fol_veh = vehicles[i + 1]
            line_segment = LineSegment(prec_veh.point_location(), fol_veh.point_location())
            point = self.point_location()

            if line_segment.is_point_projection_in(point):
                return prec_veh, fol_veh

        return -1, -1 # None, None
    


    """******************************************  Back up Functions  **********************************************************"""

    def find_look_ahead_point(self, ld):
        pass

    
    def update_lane_change_reference_path():
        pass

    def latitude_control(self):
        # Pure Pursuit algorithm: need reference line
        pass
        # k = 0.28
        # l = self.param.length
        # v = self.state.v
        # ld = 2 + k * v
        # lp = self.find_look_ahead_point(ld)
        
        # vec1 = Vector(lp - self.point_location())
        # vec2 = Vector(np.cos(self.heading), np.sin(self.heading))
        # a = angle_between_vectors(vec1, vec2)
        # steer = np.arctan(2 * l * np.sin(a) / ld)
        # self.input.steer_angle = steer

        """bicycle model"""
        # x_dot = self.state.v * np.cos(self.state.heading + self.input.steer_angle)
        # y_dot = self.state.v * np.sin(self.state.heading + self.input.steer_angle)
        # heading_dot = self.state.v * np.tan(self.input.steer_angle) / self.param.length
        # v_dot = self.input.acc

        # self.state.x += self.dt * x_dot
        # self.state.y += self.dt * y_dot
        # self.state.v += self.dt * v_dot
        # self.state.heading += self.dt * heading_dot

        # print(self)

        # self.distance += self.dt * self.velocity + self.dt * self.dt / 2.0 * self.acceleration
        # self.velocity += self.dt * self.acceleration
        # to do: update x and y, need to have the road information
        
    def update_reference_path(self):
        pass
