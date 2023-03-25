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

    def __init__(self, acc=.0, steer_angle=.0):
        self.acc = acc
        self.steer_angle = steer_angle


class Vehicle:
    acc_max = 3.0
    acc_min = -3.0
    v_max = 33.0
    v_min = 0.0

    def __init__(self, dt, simulation, v_des, permanent_id, lane, id, veh_state, veh_param, veh_input=VehicleInput(0, 0)):
        # we need some label to index the HDV lane, id....
        self.permanent_id = permanent_id
        self.dt = dt
        self.lane = lane
        self.id = id
        self.category = None
        self.simulation = simulation
        self.road = simulation.road
        # vehicle may have different segment because of different lane
        self.segment = self.road.segments[0]
        self.reference_path = None

        # vehicle physical parameters and control state/input
        self.state = veh_state
        self.param = veh_param
        self.input = veh_input
        self.v_des = v_des
        self.travel_distance = 0

        self.energy_consumption = 0

    def __str__(self):
        return f"id: {self.permanent_id}, (x, y): ({self.state.x}, {self.state.y}), v: {self.state.v}"
        # lane: {self.lane}, , head: {self.state.heading}, acc: {self.input.acc}, steer: {self.input.steer_angle}

    def update(self):
        return

    def update_labels(self):
        # TODO(Hanyu): update the labels of the vehicle including lane_idx, id, segment, road.
        pass

    def update_consumption(self):
        self.energy_consumption += self.energy_consumption_model()
        # print(f"vehicle energy consumption: {self.energy_consumption}")

    def energy_consumption_model(self):
        u = self.input.acc
        v = self.state.v
        instant_c = -753.7 + 9.7326 * v - 0.3014 * v**2 + 0.0053 * v**3 + 44.3809 * u + 5.1753 * u * v - 0.0742 * u * v**2 + 0.0006 * u * v**3 \
            + 17.1641 * u**2 + 0.2942 * u**2 * v + 0.0109 * u**2 * v**2 - 0.001 * u**2 * v**3 \
            - 4.2024 * u**3 - 0.7068 * u**3 * v + 0.0116 * u**3 * v**2 - 0.0006 * u**3 * v**3
        # print(np.exp(0.01 * instant_c))
        return np.exp(0.01 * instant_c)

    def IDM_predict(self, precede_veh):
        # if precede_veh is None:
        #     s = 100000

        v = self.state.v
        T = 1.5
        a = 0.73
        b = 1.67
        delta = 4
        l = self.param.length
        s0 = 2 + l  # buffer distance will count vehicle length
        x = self.state.x
        y = self.state.y
        s = np.sqrt((precede_veh.state.x - x)**2 + (precede_veh.state.y - y)**2) - \
            l if precede_veh is not None else 100000  # why 1 is too small
        dv = v - precede_veh.state.v if precede_veh is not None else 0.0
        s_des = s0 + v * T + v * dv / (2 * np.sqrt(a * b))
        acc = a * (1 - (v / self.v_des) ** delta - (s_des / s) ** 2)
        return acc

    def point_location(self):
        """obtain vehicle's point location"""
        return Point(self.state.x, self.state.y)
    pass
