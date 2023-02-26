import random
from Vehicle import *
from Road import *
from Simulation import *
from Controller import *


class Generator:
    """generate random vehicle at the entrance of the road: lane number, velocity"""
    # random lane number, accroding to differnet lane number, have different random generation rate and velocity
    # for each lane, generate different rates of hdvs
    generate_static_obstacles_once = 0
    num_hdvs = 0
    permanent_id = 0
    num_cavs = 0

    def __init__(self, simulation, generation_rate = 20, dt = 0.01, random_seed = 42, mode = 0):
        self.t = 0
        self.last_generation = 0
        self.dt = dt
        self.generation_rate = generation_rate
        self.simulation = simulation
        self.road = self.simulation.road
        self.random_seed = random_seed
        self.mode = mode
        random.seed(random_seed)


    def update(self):
        """Update the generator state"""
        self.t += self.dt
        if self.mode == 0:
            # single cav with static obstacles
            if self.generate_static_obstacles_once == 0:
                # generate statci obstacles
                for _ in range(10):
                    self.generate_static_obstacles()
                for _ in range(10):
                    self.generate_static_obstacles(0)
                for _ in range(10):
                    self.generate_static_obstacles(2)
                self.generate_static_obstacles_once += 1
            

            # generate one CAV
            if self.last_generation == 0 and self.t >= 1 and random.uniform(0, 1) < 0.5:
                self.last_generation += 1
                self.generate_CAV(v=20)
                print("CAV generated")

        elif self.mode == 1:
            # generate one CAV
            if self.num_cavs == 0 and self.t >= 30 and random.uniform(0, 1) < 0.5:
                self.last_generation += 1
                self.generate_CAV(v = 15)
                self.num_cavs += 1
                print("CAV generated")

            # generate moving hdvs
            if self.t - self.last_generation <= 2:
                return
            
            if self.num_hdvs >= 15:
                return

            if random.uniform(0, 1) <= 0.9:
                self.generate_HDV(v = 15, v_des = 15)
                self.last_generation = self.t
                self.num_hdvs += 1
        
        elif self.mode == 2:
            # generate multiple cavs, let them form a platoon
            if self.num_cavs < 4 and random.uniform(0, 1) < 0.5:
                self.generate_CAV(v = 15)
                self.num_cavs += 1
                print("cav generated")

    def generate_static_obstacles(self, lane_idx = 1):
        # TODO: extend to arc segment
        self.permanent_id += 1
        v = 0
        v_des = 0
        heading = 0

        segment_start = self.road.segments[0].start
        segment_end = self.road.segments[0].end

        lam = random.uniform(0, 1)
        x = lam * segment_start.x + (1 - lam) * segment_end.x
        if self.road.num_lanes % 2 == 0:
            lane_factor = lane_idx - (self.road.num_lanes + 1) // 2
        else:
            lane_factor = lane_idx - self.road.num_lanes // 2
        y = segment_start.y + lane_factor * self.road.lane_width

        veh_state = VehicleState(x, y, v, heading)
        veh_param = VehicleParameter(4, 1.5)
        hdv = HDV(self.dt, self.simulation, v_des, self.permanent_id, lane_idx, len(self.road.segments[0].vehicles[lane_idx]), veh_state, veh_param)
        self.road.add_vehicle(hdv)
        return hdv


    def generate_vehicle(self, is_cav, v = None, v_des = None, lane_idx = None):
        """Function to generate vehicle"""
        self.permanent_id += 1
        if lane_idx is None:
            lane_idx = self.init_lane_idx()
        if v is None:
            v = self.init_speed(lane_idx)
        if v_des is None:
            v_des = self.init_des_speed(lane_idx)
        heading = self.init_vehicle_heading()
        pos = self.init_vehicle_position(lane_idx)
        
        veh_state = VehicleState(*pos, v, heading)
        veh_param = VehicleParameter(4.5, 1.5) # todo: may generate vehicles with different size
        
        if is_cav:
            cav = CAV(self.dt, self.simulation, v_des, self.permanent_id, lane_idx, len(self.road.segments[0].vehicles[lane_idx]), veh_state, veh_param)
            controller = LonLatController(cav)
            cav.add_controller(controller)
            self.road.add_vehicle(cav)
            return cav
        else:
            hdv = HDV(self.dt, self.simulation, v_des, self.permanent_id, lane_idx, len(self.road.segments[0].vehicles[lane_idx]), veh_state, veh_param)
            self.road.add_vehicle(hdv)
            return hdv

    def generate_CAV(self, v = None, v_des = None, lane_idx = None):
        return self.generate_vehicle(True, v, v_des, lane_idx)

    def generate_HDV(self, v = None, v_des = None, lane_idx = None):
        return self.generate_vehicle(False, v, v_des, lane_idx)

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


