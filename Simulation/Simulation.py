# to do: the simulation of HDVs and CAVs, synchronize update time
from Road import *
from Vehicle import *
from Generator import *
import pandas as pd
import numpy as np
from Platoon import *
from datetime import datetime


class Simulation:
    """Simulation class: handle the entire simulation"""

    def __init__(self, road=None, generator=None, platoon=None):
        self.t = 0
        self.platoon_t = 0
        self.dt = 0.05
        self.count = 0

        self.road = road  # may extend to multiple roads in the future
        # may extend to multiple vehicle generators for different roads in the future
        self.generator = generator
        self.platoon = platoon

        self.init_data_record()
        self.init_traffic_flow_summary()

    def init_traffic_flow_summary(self):
        self.energy_consumption = 0
        self.traffic_speed = 0
        self.traffic_flow = 0
        self.traffic_density = 0

    def init_data_record(self):
        self.data = {}  # this is the record the data into an excel

        self.data_id = []
        self.data_veh_category = []
        self.data_time = []

        self.data_lane_idx = []
        self.data_veh_length = []
        self.data_veh_speed = []
        self.data_veh_x = []
        self.data_veh_y = []
        self.data_veh_distance = []

    def create_road(self, waypoints, init_heading=0.0, num_lanes=3, road_width=4):
        """Create one single road"""
        self.road = Road(waypoints, init_heading, num_lanes, road_width)

    def create_generator(self, mode=0, platoon=None):
        """Create a traffic generator"""
        self.generator = Generator(
            self, dt=self.dt, mode=mode, platoon=platoon)

    def create_platoon(self):
        """Create a platoon planner"""
        self.platoon = Platoon(self.road)
        self.generator.add_platoon(self.platoon)

    def update(self):
        """Update the simulation"""
        if self.count % 1 == 0:
            self.generator.update()  # update generator to generate new vehicles
        if self.count % 20 == 0:
            self.platoon.update()
        # make road to update vehicle information
        self.road.update_vehicles(self.dt)
        self.update_simulation_time()
        self.update_traffic_flow_summary()

        # record data
        self.record()

        """"********************************************************* Support Functions***************************************************************"""

    def update_simulation_time(self):
        self.t += self.dt
        self.count += 1
        self.platoon_t = self.platoon.t

    def update_traffic_flow_summary(self):
        self.road.update_traffic_flow_summary()
        self.energy_consumption = self.road.energy_consumption

    def record(self):
        """Record the traffic simulation data for further analysis"""
        if self.count % 100 != 0:
            return

        for segment in self.road.segments:
            for lane in segment.vehicles:
                for vehicle in lane:
                    self.record_vehicle(vehicle)

    def record_vehicle(self, vehicle):
        self.data_id.append(vehicle.permanent_id)
        self.data_time.append(self.t)
        self.data_veh_category.append(vehicle.category)
        self.data_lane_idx.append(vehicle.lane)
        self.data_veh_length.append(vehicle.param.length)
        self.data_veh_speed.append(vehicle.state.v)
        self.data_veh_x.append(vehicle.state.x)
        self.data_veh_y.append(vehicle.state.y)
        self.data_veh_distance.append(vehicle.travel_distance)

    def output_record_data_as_excel(self):
        self.data["time"] = self.data_time
        self.data["id"] = self.data_id
        self.data["category"] = self.data_veh_category
        self.data["x"] = self.data_veh_x
        self.data["y"] = self.data_veh_y
        self.data["distance"] = self.data_veh_distance
        self.data["speed"] = self.data_veh_speed
        self.data["length"] = self.data_veh_length
        self.data["lane"] = self.data_lane_idx

        dataframe = pd.DataFrame(self.data)
        date_time = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        writer = pd.ExcelWriter(
            f"Traffic_recorded_{date_time}.xlsx", engine="xlsxwriter")
        dataframe.to_excel(writer, sheet_name="Sheet1")
        writer.save()
