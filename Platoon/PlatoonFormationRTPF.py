from Utilities import *


class Platoon:
    def __init__(self, road):
        self.road = road
        self.cavs = []
        self.count = 0
        self.t = 0
        self.platoon_start = 0
        self.platoon_end = 0

        self.capacity = 5
        self.platoon_spacing = 60
        self.size_formed = 0
        self.is_platoon_formed = False

        self.target_movement_point = Point(100, 0)
        pass

    def update(self):
        # print(f"length of cavs added to platoon is {len(self.cavs)}")
        if self.platoon_is_formed():
            self.do_platooning_control()
            return

        # cavs will not start forming platoon

        if len(self.cavs) >= self.capacity:
            self.count += 1

        if len(self.cavs) < self.capacity or self.count <= 5:
            print("platoon formation has not started")
            return

        # start forming platoon
        # print("platoon starts forming")
        self.t += 1
        self.construct_cells()
        # call virtual platoon optimization to calcualte target cell and finalize target speed
        # alternative one: greedily find one

        # this is the optimal one
        self.platoon_look_ahead_formation()

        # this is the heuristic rule that just looks ahead 150m to find the formation point
        # just for comparison experiments

        # self.platoon_sequential_formation()

        # TODO: if the CAV sequential order changes --> what to do
        # Some detection algorithm: when following vehicle approaches leading vehicle, slow down speed
        # in target point determination algorithm, find target speed is important

    def construct_cells(self):
        self.v_free = 20
        self.T = 10
        self.cell_length = max(self.platoon_spacing *
                               self.capacity, self.T * self.v_free)
        self.cell_w_size = 10
        self.cell_l_size = self.road.num_lanes
        self.cells = [[0 for i in range(self.cell_w_size)]
                      for j in range(self.cell_l_size)]
        self.crit_density = 1 / self.cell_length
        self.target_cell_idx = None

        head_cav = self.cavs[0]
        cell_w_start = head_cav.state.x

        # initialize cell density
        for segment in self.road.segments:  # only one segment in this case
            for i in range(len(segment.vehicles)):
                lane_vehicles = segment.vehicles[i]
                for vehicle in lane_vehicles:
                    if vehicle.category == "HDV":
                        cell_w_index = int(
                            (vehicle.state.x - cell_w_start) / self.cell_length)
                        cell_l_index = i
                        if cell_w_index < self.cell_w_size:
                            # the unit of cell density is veh/m
                            self.cells[cell_l_index][cell_w_index] += 1 / \
                                self.cell_length

    def find_target_movement_point(self):
        distance_first_last = abs(self.cavs[0].state.x - self.cavs[-1].state.x)
        print(f"first and last CAV has {distance_first_last} distance")
        max_speed_diff = 10
        target_formation_time = distance_first_last / max_speed_diff
        target_formation_tick = int(target_formation_time / self.T)

        self.cells_at_target_time = self.cells.copy()

        # use ctm to update cell density at future time tick
        for t in range(target_formation_tick):
            for cell_l_idx in range(self.cell_l_size):
                for cell_w_idx in range(self.cell_w_size - 1, 0, -1):
                    self.cells[cell_l_idx][cell_w_idx] = self.cells[cell_l_idx][cell_w_idx - 1]
                # TODO: pass density genreator estimation
                self.cells[cell_l_idx][0] = 1 / self.cell_length

        # find the optimal cell to form platoon in the future
        leading_cav_least_driving_distance = 20 * target_formation_time
        w1 = 0  # int(leading_cav_least_driving_distance // self.cell_length)
        # print(f"w1 is {w1}")
        self.cell_costs = [
            [0 for i in range(self.cell_w_size)] for j in range(self.cell_l_size)]

        for cell_l_idx in range(self.cell_l_size):
            num_lane_changes = 0
            lane_traffic_costs = 0

            y = 10 + (1 - cell_l_idx) * self.road.lane_width
            # the lane index has problems
            for cav in self.cavs:
                num_lane_changes += 0 if abs(y - cav.state.y) <= 0.01 else 1
                lane_traffic_costs += 10 * \
                    self.road.congestion_in_lane(
                        2 - cell_l_idx, self.cavs[-1].state.x, self.cavs[0].state.x)

            for cell_w_idx in range(w1, self.cell_w_size):
                density_costs = 10 * \
                    self.cells[cell_l_idx][cell_w_idx] * self.cell_length
                efficiency_costs = 10 * cell_w_idx
                lane_change_costs = 1 * num_lane_changes * 10  # parameter sensitivity analysis

                # parameter sensitivity analysis
                # efficiency_costs = 0
                # lane_traffic_costs = 0
                # lane_change_costs = 0
                self.cell_costs[cell_l_idx][cell_w_idx] = density_costs + \
                    lane_change_costs + efficiency_costs + lane_traffic_costs

        min_costs = 100000
        target_idx = (-1, -1)
        for cell_l_idx in range(self.cell_l_size):
            for cell_w_idx in range(w1, self.cell_w_size):
                if self.cell_costs[cell_l_idx][cell_w_idx] <= min_costs:
                    min_costs = self.cell_costs[cell_l_idx][cell_w_idx]
                    target_idx = (cell_l_idx, cell_w_idx)

        if target_idx == (-1, -1):
            return Point(200, 0)

        w = target_idx[1]
        l = target_idx[0]
        print(f"target cell index is {w}, {l}")
        return Point((w+1) * self.cell_length, (l - 1) * self.road.lane_width)

        # for cell_l_idx in range(self.cell_l_size):
        #     for cell_w_idx in range(w1, self.cell_w_size):
        #         if self.cells[cell_l_idx][cell_w_idx] <= self.crit_density:
        #             self.target_cell_idx = (cell_l_idx, cell_w_idx)
        #             l = self.target_cell_idx[0]
        #             w = self.target_cell_idx[1]
        #             print(f"target point is {(w+1) * self.cell_length}, {(l - 1) * self.road.lane_width}")
        #             return Point((w+1) * self.cell_length, (l - 1) * self.road.lane_width)

    def find_hdv_speed_in_target_lane(self, lane_idx):
        pass

    def platoon_look_ahead_formation(self):
        # TODO: if the CAV sequential order changes --> what to do
        # Some detection algorithm: when following vehicle approaches leading vehicle, slow down speed
        # in target point determination algorithm, find target speed is important
        for cav in self.cavs:
            cav.is_platooning = 1  # 1 represents the cav is formation platoon, 0 represents the cav is not formation platoon, 2 represents cav is in platoon

        # find an empty space road to construct platoon
        target_movement_point = self.find_target_movement_point()

        head_cav = self.cavs[0]
        head_cav.target_location = Point(head_cav.point_location(
        ).x, self.road.segments[0].start.y) + target_movement_point  # head_cav.target_movement_point

        head_cav_prec_veh = head_cav.find_prec_veh()
        print(f"head prec veh is {head_cav_prec_veh}")
        head_cav.target_speed = 30 if head_cav_prec_veh is None or distance(
            head_cav_prec_veh.point_location(), head_cav.point_location()) > 50 else head_cav_prec_veh.state.v
        head_cav.target_time = distance(head_cav.target_location, head_cav.point_location(
        )) / (head_cav.target_speed + head_cav.state.v) * 2
        dv = 2
        for i in range(1, len(self.cavs)):
            self.cavs[i].target_location = head_cav.target_location - \
                Point(self.platoon_spacing, 0) * i
            distance_to_target = distance(
                self.cavs[i].target_location, self.cavs[i].point_location())
            self.cavs[i].target_time = head_cav.target_time
            self.cavs[i].target_speed = distance_to_target / \
                head_cav.target_time * 2 - self.cavs[i].state.v
            print(f"target speed is {self.cavs[i].target_speed}")

    def platoon_sequential_formation(self):
        for cav in self.cavs:
            cav.is_platooning = 1  # 1 represents the cav is formation platoon, 0 represents the cav is not formation platoon, 2 represents cav is in platoon

        head_cav = self.cavs[0]
        head_cav.target_location = Point(head_cav.point_location(
        ).x, self.road.segments[0].start.y) + head_cav.target_movement_point
        head_cav.target_speed = 25
        head_cav.target_time = distance(head_cav.target_location, head_cav.point_location(
        )) / (head_cav.target_speed + head_cav.state.v) * 2
        dv = 2
        for i in range(1, len(self.cavs)):
            self.cavs[i].target_location = head_cav.target_location - \
                Point(50, 0) * i
            distance_to_target = distance(
                self.cavs[i].target_location, self.cavs[i].point_location())
            self.cavs[i].target_time = head_cav.target_time
            self.cavs[i].target_speed = distance_to_target / \
                head_cav.target_time * 2 - self.cavs[i].state.v

    def add_cav(self, cav):
        self.cavs.append(cav)

    def platoon_is_formed(self):
        if self.is_platoon_formed:
            return True

        if len(self.cavs) < self.capacity:
            return False

        for i in range(1, len(self.cavs)):
            prec_cav = self.cavs[i-1]
            ego_cav = self.cavs[i]
            if distance(ego_cav.point_location(), prec_cav.point_location()) > 70 \
                    or abs(ego_cav.point_location().y - prec_cav.point_location().y) > 0.01:
                return False

        # notify platoon is formed, switch to platooning control
        self.is_platoon_formed = True
        for cav in self.cavs:
            cav.is_platooning = 2
        return True

    def do_platooning_control(self):
        print(f"do platooning control")
        # for cav in self.cavs:
        #     #TODO: acc platooning control
        #     prec_veh = cav.find_prec_veh()
        #     cav.state.v =
        #     cav.state.x += cav.dt * cav.state.v
        #     cav.input.steer_angle = 0
        #     cav.state.heading = 0
