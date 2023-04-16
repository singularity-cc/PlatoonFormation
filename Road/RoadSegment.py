import numpy as np
from Utilities import *
from Vehicle import *

class SegmentPoint:
    def __init__(self, x, y, heading = None):
        self.x = x
        self.y = y
        self.heading = heading

    def convert_to_point(self):
        return Point(self.x, self.y)

    def __str__(self):
        return f"({self.x}, {self.y}, {self.heading})"

class Segment:
    def __init__(self, start: Point, end: Point, start_heading, num_lanes, lane_width):
        # straigt segment information
        self.start = SegmentPoint(start.x, start.y, start_heading)
        self.end = SegmentPoint(end.x, end.y, start_heading)
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        
        # arc segment information
        self.arc_radius = None
        self.arc_center = None
        self.arc_start_angle = None
        self.arc_end_angle = None
        self.mid = None
        self.is_clockwise = None
        
        # initialzie arc segment
        self.init_arc_segment()

        # hold vehicles
        self.vehicles = [[] for _ in range(num_lanes)]

    def __str__(self):
        return f"{self.start}, {self.end}"

    """****************************************** Function to add/remove vehicle and update vehicle states  **********************************************************"""

    def add_vehicle(self, vehicle):
        """Add one vehicle in the current road segment"""
        self.vehicles[vehicle.lane].append(vehicle)

    def update_vehicles(self):
        """Update all vehicles' states in the current road segment"""
        for i in range(self.num_lanes):
            self.reorganize_vehicles_in_lane(i)

        # print("segment is updated")
        for vehicles_in_lane in self.vehicles:
            for vehicle in vehicles_in_lane:
                # print("vehicle in segment is updated")
                # print(f"vehicle is registered, id is {vehicle.lane}")
                vehicle.update()

    def remove_vehicle(self, vehicle):
        """Remove one vehicle from the current road segment"""
        self.vehicles[vehicle.lane].remove(vehicle)

    def reorganize_vehicles_in_lane(self, lane):
        """Reorganize all the vehicles in the target lane in the current segment by reassigning the vehicle id"""
        id = 0
        #TODO: extend to curvature road
        self.vehicles[lane].sort(key=lambda veh: veh.travel_distance, reverse=True)
        for vehicle in self.vehicles[lane]:
            vehicle.id = id
            id += 1

    def update_vehicle_lane(self, vehicle, is_left):
        """Update the vehicle lane index: remove/add vehicle, reorganize vehicles in original/new lane"""
        self.remove_vehicle(vehicle)

        original_lane = vehicle.lane
        # if (is_left and not self.is_clockwise) or (not is_left and self.is_clockwise):
        if is_left:
            vehicle.lane -= 1
        else:
            vehicle.lane += 1

        self.add_vehicle(vehicle)
        self.reorganize_vehicles_in_lane(original_lane)
        self.reorganize_vehicles_in_lane(vehicle.lane)
    
    
    """****************************************** Function to initialize a segment  **********************************************************"""

    def init_arc_segment(self):
        """Initialize an arc segment"""
        # edge case: segment is straight
        if self.is_segment_straight_getter():
            return

        # calculate and assign the arc center and radius
        self.arc_center = self.arc_center_getter()
        self.arc_radius = self.arc_radius_getter()

        # calculate the heading angle of the end segment point
        self.end.heading = self.end_heading_getter()

        # calcualte the arc start and end angles for the segment
        self.arc_start_angle = self.arc_start_angle_getter()
        self.arc_end_angle = self.arc_end_angle_getter()

        # calcualte the arc direction: clockwise or anti-clockwise 
        self.is_clockwise = self.is_clockwise_getter()

    """****************************************** Getter functions to obtain certain data members  **********************************************************"""
    def is_segment_straight_getter(self):
        """Return if the current road segment is straight or not"""
        return self.start_heading_vec_getter().is_parallel(self.start_end_vec_getter())

    def arc_center_getter(self):
        """Return the arc center of the current road segment"""
        return intersect(self.start_center_line_getter(), self.mid_center_line_getter())

    def arc_radius_getter(self):
        """Return the arc radius of the current road segment"""
        return np.sqrt((self.start.x - self.arc_center.x) ** 2 + (self.start.y - self.arc_center.y) ** 2)

    def arc_end_angle_getter(self):
        """Return the end point angle of the arc in the current road segment"""
        return Vector(self.end.x - self.arc_center.x, self.end.y - self.arc_center.y).convert_to_angle()

    def arc_start_angle_getter(self):
        """Return the start point angle of the arc in the current road segment"""
        return Vector(self.start.x - self.arc_center.x, self.start.y - self.arc_center.y).convert_to_angle()

    def end_heading_getter(self):
        """Return the end point heading of the current road segment"""
        if self.start_heading_vec_getter().inner_product(self.start_end_vec_getter()) == 0:
            extension_point = self.end.convert_to_point() - self.start_heading_vec_getter()
        elif self.start_heading_vec_getter().inner_product(self.start_end_vec_getter()) > 0:
            extension_point = self.end.convert_to_point() + self.start_end_vec_getter()
        elif self.start_heading_vec_getter().inner_product(self.start_end_vec_getter()) < 0:
            extension_point = self.end.convert_to_point() - self.start_end_vec_getter()

        project_point = self.end_center_line_getter().projection(extension_point)
        end_head_vec = Vector(extension_point.x - project_point.x, extension_point.y - project_point.y)
        return end_head_vec.convert_to_angle()

    def is_clockwise_getter(self):
        """Return if the current road segment is clockwise or not"""
        start_mid_vec = Vector(self.mid_arc_point_getter().x - self.start.x, self.mid_arc_point_getter().y - self.start.y)
        start_end_vec = Vector(self.end.x - self.start.x, self.end.y - self.start.y)
        return start_end_vec.cross_product(start_mid_vec) < 0

    """****************************************** Support Functions  **********************************************************"""
    def mid_center_line_getter(self):
        mid_center_line = Line()
        mid_center_line.init_norm(self.start_end_vec_getter(), self.mid_line_point_getter())
        return mid_center_line

    def mid_arc_point_getter(self):
        if self.is_segment_straight_getter():
            return None

        center_mid_vec = Vector(self.mid_line_point_getter().x - self.arc_center.x, self.mid_line_point_getter().y - self.arc_center.y)
        if center_mid_vec.is_empty():
            # mid and center collides
            center_mid_vec = self.start_heading_vec_getter()

        center_mid_vec.normalize()
        if self.start_heading_vec_getter().inner_product(self.start_end_vec_getter()) >= 0:
            return self.arc_center +  center_mid_vec * self.arc_radius
        return self.arc_center - center_mid_vec * self.arc_radius

    def end_center_line_getter(self):
        end_center_line = Line()
        end_center_line.init_line(self.end_norm_vec_getter(), self.end)
        return end_center_line

    def start_center_line_getter(self):
        start_center_line = Line()
        start_center_line.init_norm(self.start_heading_vec_getter(), self.start)
        return start_center_line

    def mid_line_point_getter(self):
        return Point((self.start.x + self.end.x) / 2, (self.start.y + self.end.y) / 2)

    def start_end_vec_getter(self):
        return Vector(self.end.x - self.start.x, self.end.y - self.start.y)

    def start_heading_vec_getter(self):
        return Vector(np.cos(self.start.heading / 180 * np.pi), np.sin(self.start.heading / 180 * np.pi))

    def end_norm_vec_getter(self):
        return Vector(self.arc_center.x - self.end.x, self.arc_center.y - self.end.y)
