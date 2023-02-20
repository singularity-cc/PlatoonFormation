from Utilities import *

class VehicleState:
    """Vehicle state including position, speed, heading"""
    def __init__(self, x, y, v, heading):
        self.x = x
        self.y = y
        self.v = v
        self.heading = heading

class VehicleParameter:
    """Vehicle physical parameter including vehicle length, vehicle width"""
    def __init__(self, veh_length, veh_width):
        self.length = veh_length
        self.width = veh_width

class VehicleInput:
    """Vehicle control input including longitudinal acceleration and lateral steering angle"""
    def __init__(self, acc = .0, steer_angle = .0):
        self.acc = acc
        self.steer_angle = steer_angle


class Vehicle:
    acc_max = 3.0
    acc_min = -3.0
    v_max = 33.0
    v_min = 0.0

    def __init__(self, dt, simulation, v_des, permanent_id, lane, id, veh_state, veh_param, veh_input = VehicleInput(0, 0)):
        # we need some label to index the HDV lane, id....
        self.permanent_id = permanent_id
        self.dt = dt
        self.lane = lane
        self.id = id
        self.category = None
        self.simulation = simulation
        self.road = simulation.road
        self.segment = self.road.segments[0] # vehicle may have different segment because of different lane
        self.reference_path = None

        # vehicle physical parameters and control state/input
        self.state = veh_state
        self.param = veh_param
        self.input = veh_input
        self.v_des = v_des
        self.travel_distance = 0

        self.energy_consumption = 0

    def __str__(self):
        return f"id: {self.id}, lane: {self.lane}, (x, y): ({self.state.x}, {self.state.y}), v: {self.state.v}, head: {self.state.heading}, acc: {self.input.acc}, steer: {self.input.steer_angle}"


    def update(self):
        return


    def update_consumption(self):
        self.energy_consumption += self.energy_consumption_model()
        print(f"vehicle energy consumption: {self.energy_consumption}")


    def energy_consumption_model(self):
        u = self.input.acc
        v = self.state.v
        instant_c = -753.7 + 9.7326 * v - 0.3014 * v**2 + 0.0053 * v**3 + 44.3809 * u + 5.1753 * u * v - 0.0742 * u * v**2 + 0.0006 * u * v**3 \
        + 17.1641 * u**2 + 0.2942 * u**2 * v + 0.0109 * u**2 * v**2 - 0.001 * u**2 * v**3 \
        -4.2024 * u**3 - 0.7068 * u**3 * v + 0.0116 * u**3 * v**2 - 0.0006 * u**3 * v**3
        print(np.exp(0.01 * instant_c))
        return np.exp(0.01 * instant_c)
    
        
    def point_location(self):
        """obtain vehicle's point location"""
        return Point(self.state.x, self.state.y)
    pass