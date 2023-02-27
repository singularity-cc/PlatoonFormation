from Utilities import *

class Platoon:
    def __init__(self, road):
        self.road = road
        self.cavs = []
        self.capacity = 8
        self.size_formed = 0
        self.is_platoon_formed = False

        self.target_movement_point = Point(100, 0)
        pass

    def update(self):
        print(f"length of cavs added to platoon is {len(self.cavs)}")
        if self.platoon_is_formed():
            self.do_platooning_control()
            return
        
        # cavs will not start forming platoon
        if len(self.cavs) < self.capacity:
            return

        # start forming platoon
        print("platoon starts forming")
        for cav in self.cavs:
            cav.is_platooning = 1 # 1 represents the cav is formation platoon, 0 represents the cav is not formation platoon, 2 represents cav is in platoon

        head_cav = self.cavs[0]
        head_cav.target_location = Point(head_cav.point_location().x, self.road.segments[0].start.y) + head_cav.target_movement_point
        head_cav.target_speed = 25
        head_cav.target_time = distance(head_cav.target_location, head_cav.point_location()) / (head_cav.target_speed + head_cav.state.v) * 2
        dv = 1
        for i in range(1, len(self.cavs)):
            self.cavs[i].target_location = head_cav.target_location - Point(50, 0) * i
            self.cavs[i].target_speed = head_cav.target_speed + dv * i
            self.cavs[i].target_time = head_cav.target_time
        pass

    
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
            if distance(ego_cav.point_location(), prec_cav.point_location()) > 60 \
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


