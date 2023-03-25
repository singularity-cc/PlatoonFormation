from Vehicle import *
from Utilities import *
import numpy as np
from Road import *
import random


class LaneChangeController:

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.init_lane_change_variables()

    def init_lane_change_variables(self):
        """Initialize lane change data members"""
        self.vehicle.is_in_lane_change = False
        self.vehicle.lane_change_duration = None
        self.vehicle.lane_change_turn_left = None
        self.vehicle.lane_idx_change = 0
        self.vehicle.lane_change_timer = None
        self.vehicle.lane_change_target_state = None
        self.vehicle.lane_change_dx = None
        self.vehicle.lane_change_dy = None
        self.vehicle.lane_change_dheading = None

    def update(self):
        self.update_lane_change_decision()

    def update_lane_change_decision(self):
        """No additional lane change if the vehicle is doing lane change"""
        if self.vehicle.is_in_lane_change:
            return

        """Do lane change decision using random algorithm"""
        # self.MOBIL_lane_change()
        self.random_lane_change()

    def MOBIL_lane_change(self):
        """MOBIL lane change algorithm to do lane change decisions by calculating the incentives"""
        precede_veh = self.vehicle.segment.vehicles[self.vehicle.lane][self.vehicle.id -
                                                                       1] if self.vehicle.id >= 1 else None
        follow_veh = self.vehicle.segment.vehicles[self.vehicle.lane][self.vehicle.id + 1] if self.vehicle.id + 1 < len(
            self.vehicle.segment.vehicles[self.vehicle.lane]) else None

        left_precede_veh, left_follow_veh = self.surround_vehicles_in_lane(
            self.vehicle.lane + 1) if self.vehicle.segment.is_clockwise else self.surround_vehicles_in_lane(self.vehicle.lane - 1)
        right_precede_veh, right_follow_veh = self.surround_vehicles_in_lane(
            self.vehicle.lane - 1) if self.vehicle.segment.is_clockwise else self.surround_vehicles_in_lane(self.vehicle.lane + 1)

        left_lane_change_benefit = -1
        right_lane_change_benefit = -1

        ego_a_no_lane_change = self.vehicle.IDM_predict(precede_veh)
        fol_a1_no_lane_change = follow_veh.IDM_predict(
            self) if follow_veh is not None else 0

        if self.is_gap_acceptable(left_precede_veh, left_follow_veh) and left_precede_veh != -1:
            ego_a_after_lane_change = self.vehicle.IDM_predict(
                left_precede_veh)
            fol_a1_after_lane_change = follow_veh.IDM_predict(
                precede_veh) if follow_veh is not None else 0
            fol_a2_no_lane_change = left_follow_veh.IDM_predict(
                left_precede_veh)
            fol_a2_after_lane_change = left_follow_veh.IDM_predict(
                self.vehicle)
            left_lane_change_benefit = ego_a_after_lane_change + fol_a1_after_lane_change + \
                fol_a2_after_lane_change - ego_a_no_lane_change - \
                fol_a1_no_lane_change - fol_a2_no_lane_change
        if self.is_gap_acceptable(right_precede_veh, right_follow_veh) and right_precede_veh != -1:
            ego_a_after_lane_change = self.vehicle.IDM_predict(
                right_precede_veh)
            fol_a1_after_lane_change = follow_veh.IDM_predict(
                precede_veh) if follow_veh is not None else 0
            fol_a2_no_lane_change = right_follow_veh.IDM_predict(
                right_precede_veh)
            fol_a2_after_lane_change = right_follow_veh.IDM_predict(
                self.vehicle)
            right_lane_change_benefit = ego_a_after_lane_change + fol_a1_after_lane_change + \
                fol_a2_after_lane_change - ego_a_no_lane_change - \
                fol_a1_no_lane_change - fol_a2_no_lane_change

        if left_lane_change_benefit > right_lane_change_benefit > 0:
            self.init_lane_change(is_left=True)
        elif right_lane_change_benefit > left_lane_change_benefit > 0:
            self.init_lane_change(is_left=False)

        pass

    def random_lane_change(self):
        """Random Lane change algorithm to do lane change decisions randomly once the gap is acceptable"""
        precede_veh = self.vehicle.segment.vehicles[self.vehicle.lane][self.vehicle.id -
                                                                       1] if self.vehicle.id >= 1 else None
        follow_veh = self.vehicle.segment.vehicles[self.vehicle.lane][self.vehicle.id + 1] if self.vehicle.id + 1 < len(
            self.vehicle.segment.vehicles[self.vehicle.lane]) else None

        left_precede_veh, left_follow_veh = self.surround_vehicles_in_lane(
            self.vehicle.lane + 1) if self.vehicle.segment.is_clockwise else self.surround_vehicles_in_lane(self.vehicle.lane - 1)
        right_precede_veh, right_follow_veh = self.surround_vehicles_in_lane(
            self.vehicle.lane - 1) if self.vehicle.segment.is_clockwise else self.surround_vehicles_in_lane(self.vehicle.lane + 1)

        # self.lane >= 1 and
        if self.is_gap_acceptable(left_precede_veh, left_follow_veh) and random.uniform(0, 1) < 0.2:
            self.init_lane_change(is_left=True)

        # self.lane < self.segment.num_lanes - 1 and
        elif self.is_gap_acceptable(right_precede_veh, right_follow_veh) and random.uniform(0, 1) < 0.5:
            self.init_lane_change(is_left=False)

    def init_lane_change(self, is_left):
        """# initilaize the lane change maneuver: set up lane change direction (left or right lane change), lane change duration, target state after lane change, etc."""
        if self.vehicle.is_in_lane_change == False:
            self.vehicle.lane_change_turn_left = is_left
            self.vehicle.lane_idx_change = -1 if is_left else 1
            self.vehicle.is_in_lane_change = True
            self.vehicle.lane_change_duration = random.uniform(0.3, 1.5)
            self.vehicle.lane_change_timer = 0
            self.find_lane_change_target_state()
            self.calculate_lane_change_state_change()

    def do_lane_change(self):
        """# do lane change maneuver: update vehicle states during the lane change"""
        self.vehicle.state.x += self.vehicle.lane_change_dx
        self.vehicle.state.y += self.vehicle.lane_change_dy
        self.vehicle.state.heading += self.vehicle.lane_change_dheading

    def find_lane_change_target_state_in_straight_road(self):
        """ find target state after lane change in straight road """
        d = self.vehicle.state.v * self.vehicle.lane_change_duration
        theta = np.arctan(self.vehicle.road.lane_width / d)
        vec_direction = Vector(np.cos(
            self.vehicle.state.heading / 180.0 * np.pi), np.sin(self.vehicle.state.heading / 180.0 * np.pi))
        vec_direction.normalize()
        if self.vehicle.lane_change_turn_left:
            vec_direction.clockwise_rotate(-theta)
        else:
            vec_direction.anti_clockwise_rotate(-theta)
        self.vehicle.lane_change_target_state = VehicleState(
            self.vehicle.state.x + d * vec_direction.x, self.vehicle.state.y + d * vec_direction.y, self.vehicle.state.v, self.vehicle.state.heading)

    def find_lane_change_target_state_in_arc_road(self):
        """ find target state after lane change in arc road """
        d = self.vehicle.state.v * self.vehicle.lane_change_duration
        theta = d / self.vehicle.segment.arc_radius
        center_to_veh_vector = Vector(
            self.vehicle.state.x - self.vehicle.segment.arc_center.x, self.vehicle.state.y - self.vehicle.segment.arc_center.y)
        center_to_veh_vector.normalize()
        lane_width = self.vehicle.road.lane_width
        num_lanes = self.vehicle.road.num_lanes
        lane_factor = self.vehicle.lane - (num_lanes // 2)
        radius = self.vehicle.segment.arc_radius + lane_width * lane_factor

        if self.vehicle.lane_change_turn_left:
            if self.vehicle.segment.is_clockwise:
                print(center_to_veh_vector)
                center_to_veh_vector.anti_clockwise_rotate(theta)
                radius += lane_width
                target = VehicleState(self.vehicle.segment.arc_center.x + radius * center_to_veh_vector.x,
                                      self.vehicle.segment.arc_center.y + radius * center_to_veh_vector.y, self.vehicle.state.v, self.vehicle.state.heading + theta / np.pi * 180.0)
            else:
                center_to_veh_vector.clockwise_rotate(theta)
                radius -= lane_width
                target = VehicleState(self.vehicle.segment.arc_center.x + radius * center_to_veh_vector.x,
                                      self.vehicle.segment.arc_center.y + radius * center_to_veh_vector.y, self.vehicle.state.v, self.vehicle.state.heading - theta / np.pi * 180.0)
        else:
            if self.vehicle.segment.is_clockwise:
                center_to_veh_vector.anti_clockwise_rotate(theta)
                radius -= lane_width
                target = VehicleState(self.vehicle.segment.arc_center.x + radius * center_to_veh_vector.x,
                                      self.vehicle.segment.arc_center.y + radius * center_to_veh_vector.y, self.vehicle.state.v, self.vehicle.state.heading + theta / np.pi * 180.0)
            else:
                center_to_veh_vector.clockwise_rotate(theta)
                radius += lane_width
                target = VehicleState(self.vehicle.segment.arc_center.x + radius * center_to_veh_vector.x,
                                      self.vehicle.segment.arc_center.y + radius * center_to_veh_vector.y, self.vehicle.state.v, self.vehicle.state.heading - theta / np.pi * 180.0)

        self.vehicle.lane_change_target_state = target

    def find_lane_change_target_state(self):
        """find the target state after lane change, two scnearios: (1) lane change in straight road, (2) lane change in arc road """
        if self.vehicle.segment.is_segment_straight_getter():
            self.find_lane_change_target_state_in_straight_road()
        else:
            self.find_lane_change_target_state_in_arc_road()

    def calculate_lane_change_state_change(self):
        """ calculate the lane change state change during one simulation time interval (self.dt) """
        time_intervals = self.vehicle.lane_change_duration / self.vehicle.dt
        self.vehicle.lane_change_dx = (
            self.vehicle.lane_change_target_state.x - self.vehicle.state.x) / time_intervals
        self.vehicle.lane_change_dy = (
            self.vehicle.lane_change_target_state.y - self.vehicle.state.y) / time_intervals
        self.vehicle.lane_change_dheading = (
            self.vehicle.lane_change_target_state.heading - self.vehicle.state.heading) / time_intervals

    def distance_to_vehicle(self, veh):
        """check Euclidean distance between two vehicles"""
        return np.sqrt((veh.state.x - self.vehicle.state.x)**2 + (veh.state.y - self.vehicle.state.y)**2)

    def is_gap_acceptable(self, prec_veh, fol_veh):
        """check if the lane change gap is acceptable"""
        if prec_veh == -1 or fol_veh == -1:
            return False

        T = 1.5
        condition1 = self.distance_to_vehicle(
            prec_veh) >= self.vehicle.param.length + T * self.vehicle.state.v if prec_veh is not None else True
        condition2 = self.distance_to_vehicle(
            fol_veh) >= self.vehicle.param.length + T * fol_veh.state.v if fol_veh is not None else True

        return condition1 and condition2

    def surround_vehicles_in_lane(self, lane_idx):
        """obtain ego vehicle's surrounding vehicles in another lane to get prepared for lane change"""
        if lane_idx < 0 or lane_idx > self.vehicle.segment.num_lanes - 1:
            return -1, -1

        vehicles = self.vehicle.segment.vehicles[lane_idx]
        for i in range(len(vehicles) - 1):
            prec_veh = vehicles[i]
            fol_veh = vehicles[i + 1]
            line_segment = LineSegment(
                prec_veh.point_location(), fol_veh.point_location())
            point = self.vehicle.point_location()

            if line_segment.is_point_projection_in(point):
                return prec_veh, fol_veh

        return -1, -1  # None, None
