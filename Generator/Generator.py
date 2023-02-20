import random
from Vehicle import *
from Road import *
from Simulation import *


class Generator:
    """generate random vehicle at the entrance of the road: lane number, velocity"""
    # random lane number, accroding to differnet lane number, have different random generation rate and velocity
    # for each lane, generate different rates of hdvs

    permanent_id = 0

    def __init__(self, simulation, generation_rate = 20, dt = 0.01, random_seed = 42):
        self.t = 0
        self.last_generation = 0
        self.dt = dt
        self.generation_rate = generation_rate
        self.simulation = simulation
        self.road = self.simulation.road
        self.random_seed = random_seed
        random.seed(random_seed)

    def update(self):
        """Update the generator state"""
        self.t += self.dt

        # CAV TEST
 
        if self.last_generation == 0 and self.t >= 1 and random.uniform(0, 1) < 0.5:
            self.last_generation += 1
            self.generate_CAV()
            print("CAV generated")

        # if self.t - self.last_generation <= 1.5:
        #     return
        
        # if random.uniform(0, 1) <= 0.9:
        #     self.generate_HDV()
        #     self.last_generation = self.t


    def generate_vehicle(self, is_cav):
        """Function to generate vehicle"""
        self.permanent_id += 1

        lane_idx = self.init_lane_idx()
        v = self.init_speed(lane_idx)
        v_des = self.init_des_speed(lane_idx)
        pos = self.init_vehicle_position(lane_idx)
        heading = self.init_vehicle_heading()
        
        veh_state = VehicleState(*pos, v, heading)
        veh_param = VehicleParameter(4.5, 1.5) # todo: may generate vehicles with different size
        
        if is_cav:
            cav = CAV(self.dt, self.simulation, v_des, self.permanent_id, lane_idx, len(self.road.segments[0].vehicles[lane_idx]), veh_state, veh_param)
            self.road.add_vehicle(cav)
            return cav
        else:
            hdv = HDV(self.dt, self.simulation, v_des, self.permanent_id, lane_idx, len(self.road.segments[0].vehicles[lane_idx]), veh_state, veh_param)
            self.road.add_vehicle(hdv)
            return hdv

    def generate_CAV(self):
        return self.generate_vehicle(is_cav = True)

    def generate_HDV(self):
        return self.generate_vehicle(is_cav = False)

    """****************************************** Vehicle Initialization Functions **********************************************************"""
    def init_lane_idx(self):
        """Initialize vehicle lane index"""
        num_lanes = self.road.num_lanes
        return random.randrange(0, num_lanes)   
        
    def init_vehicle_heading(self):
        """Initialize vehicle heading"""
        return self.road.init_heading

    def init_vehicle_position(self, lane_idx):
        """Initialize vehicle position"""
        init_angle = self.init_vehicle_heading()/ 180 * np.pi
        start_cos, start_sin = np.cos(init_angle), np.sin(init_angle)
        segment_point = self.road.segments[0].start
        init_lane_dx, init_lane_dy = self.road.lane_width * start_sin, self.road.lane_width * start_cos
        if self.road.num_lanes % 2 == 0:
            lane_factor = lane_idx - (self.road.num_lanes + 1) // 2
        else:
            lane_factor = lane_idx - self.road.num_lanes // 2

        x = segment_point.x + lane_factor * init_lane_dx
        y = segment_point.y - lane_factor * init_lane_dy
        return (x, y)

    def init_des_speed(self, lane_idx):
        """Initialize vehicle desired speed"""
        # todo: may set more realistic algorithm
        return random.uniform(24, 30)
        # if (random.uniform(0, 1) >= 0.8):
        #     return random.uniform(20, 30)

        # if lane_idx == self.road.num_lanes - 1:
        #     v = random.uniform(15, 22)
        # elif lane_idx == self.road.num_lanes - 2:
        #     v = random.uniform(22, 26)
        # elif lane_idx == self.road.num_lanes - 3:
        #     v = random.uniform(26, 32)
        # else:
        #     v = random.uniform(30, 32)
        # return v


    def init_speed(self, lane_idx):
        """Initialize vehicle speed"""
        return random.uniform(24, 30)
        # if lane_idx == self.road.num_lanes - 1:
        #     v = random.uniform(15, 22)
        # elif lane_idx == self.road.num_lanes - 2:
        #     v = random.uniform(22, 26)
        # elif lane_idx == self.road.num_lanes - 3:
        #     v = random.uniform(26, 32)
        # else:
        #     v = random.uniform(30, 32)
        # return v


