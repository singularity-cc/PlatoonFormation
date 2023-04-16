# TODO(hanyu): implement a vehicle plant for HDV and CV that the vehicles can follow the lane or lane change decisions accurately
from Vehicle import *
from Utilities import *
import numpy as np


class LongitudinalVehiclePlant:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def update(self):
        """Update the HDV states including speed, position, lane change state, segment and road"""
        self.update_speed()
        self.update_position()
        self.update_state_after_lane_change()
        self.update_segment()
        self.update_road()

    def update_segment(self):
        """Todo: Update the HDV road segment information"""
        segment_end_point = self.vehicle.segment.end.convert_to_point()
        vehicle_current_point = self.vehicle.point_location()
        # vehicle_previous_point = Point(self.vehicle.prev_state.x, self.vehicle.prev_state.y)
        # line_segment = LineSegment(vehicle_current_point, vehicle_previous_point)

        if distance(vehicle_current_point, segment_end_point) < self.vehicle.road.lane_width * self.vehicle.road.num_lanes:
            self.vehicle.segment.remove_vehicle(self.vehicle)
            self.vehicle.segment.reorganize_vehicles_in_lane(self.vehicle.lane)

        # if line_segment.is_point_projection_in(segment_end_point):
        #     self.vehicle.segment.remove_vehicle(self)

    def update_road(self):
        """Todo: Update the HDV road information"""
        pass

    def update_state_after_lane_change(self):
        """Update the HDV lane-change state and lane index and id after lane change"""
        if self.vehicle.is_in_lane_change and self.vehicle.lane_change_timer >= self.vehicle.lane_change_duration:
            self.vehicle.is_in_lane_change = False  # lane change finishes
            self.update_lane_and_id()

    def update_speed(self):
        """Update the HDV speed information"""
        self.vehicle.state.v += self.vehicle.dt * self.vehicle.input.acc

    def update_position(self):
        prev_pos = Point(self.vehicle.state.x, self.vehicle.state.y)
        """Update the HDV position information"""
        if self.vehicle.is_in_lane_change:
            self.update_position_in_lane_change()
        else:
            self.update_position_in_car_following()
        cur_pos = Point(self.vehicle.state.x, self.vehicle.state.y)

        self.vehicle.travel_distance += distance(prev_pos, cur_pos)

    def update_lane_and_id(self):
        """Update the HDV lane and id after lane change"""
        pass
        # self.vehicle.segment.update_vehicle_lane(
        #     self.vehicle, self.vehicle.lane_change_turn_left)

    def update_position_in_lane_change(self):
        """Update the HDV position during lane change"""
        self.do_lane_change()
        self.vehicle.lane_change_timer += self.vehicle.dt

    def update_position_in_car_following(self):
        """Update the HDV position during car following"""
        d = self.vehicle.state.v * self.vehicle.dt
        if self.vehicle.segment.is_segment_straight_getter():
            """if current road segment is straight"""
            vec_direction = Vector(self.vehicle.segment.end.x - self.vehicle.segment.start.x,
                                   self.vehicle.segment.end.y - self.vehicle.segment.start.y)
            vec_direction.normalize()
            self.vehicle.state.x += d * vec_direction.x
            self.vehicle.state.y += d * vec_direction.y
        else:
            """if current road segment is an arc"""
            lane_width = self.vehicle.road.lane_width
            num_lanes = self.vehicle.road.num_lanes
            lane_factor = self.vehicle.lane - (num_lanes // 2)
            radius = self.vehicle.segment.arc_radius + lane_width * lane_factor
            theta = d / radius
            vec_direction = Vector(np.cos(self.vehicle.state.heading / 180.0 * np.pi),
                                   np.sin(self.vehicle.state.heading / 180.0 * np.pi))
            vec_direction.normalize()
            self.vehicle.state.x += d * vec_direction.x
            self.vehicle.state.y += d * vec_direction.y
            self.vehicle.state.heading += theta / np.pi * 180.0  # clockwise or anticlockwise

    def do_lane_change(self):
        """# do lane change maneuver: update vehicle states during the lane change"""
        self.vehicle.state.x += self.vehicle.lane_change_dx
        self.vehicle.state.y += self.vehicle.lane_change_dy
        self.vehicle.state.heading += self.vehicle.lane_change_dheading
