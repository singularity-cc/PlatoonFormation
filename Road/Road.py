from .RoadSegment import *

class Road:
    def __init__(self, waypoints = [], init_heading = 0.0, num_lanes = 3, lane_width = 4):
        self.segments = []
        self.lane_width = lane_width
        self.num_lanes = num_lanes
        self.init_heading = init_heading

        self.init_segments(waypoints, init_heading, num_lanes)  

        self.init_traffic_flow_summary()

    def init_traffic_flow_summary(self):
        self.energy_consumption = 0
        self.traffic_flow = 0
        self.traffic_speed = 0
        self.traffic_density = 0
    
    def init_segments(self, waypoints, init_heading, num_lanes):
        start_heading = init_heading
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            segment = Segment(start, end, start_heading, num_lanes)
            self.segments.append(segment)
            # update the heading of the start waypoint for the next segment
            start_heading = segment.end.heading

    def __str__(self):
        display = ""
        for segment in self.segments:
            display += f"[{segment}]; "
        return display

    def add_vehicle(self, vehicle):
        """Add vehicles to the current road"""
        if self.segments == None:
            raise Exception("Road is empty")

        self.segments[0].add_vehicle(vehicle)

    def update_vehicles(self, dt):
        """Update vehicle states in the current road"""
        for segment in self.segments:
            # print("road update is called")
            segment.update_vehicles()

    def update_traffic_flow_summary(self):
        self.energy_consumption = 0
        for segment in self.segments:
            for lane in segment.vehicles:
                for vehicle in lane:
                    self.energy_consumption += vehicle.energy_consumption

    def in_same_lane(self, point1, point2):
        # TODO: Extend the function to more road geometry
        # for straight road, y is constant
        if abs(point1.y - point2.y) < self.lane_width / 2:
            return True
        return False

    def congestion_in_lane(self, lane_idx, start_x, end_x):
        num_hdvs = 0
        for segment in self.segments:
            for vehicle in segment.vehicles[lane_idx]:
                if vehicle.category == "HDV":
                    if vehicle.state.x <= end_x and vehicle.state.x >= start_x:
                        num_hdvs += 1

        return num_hdvs





