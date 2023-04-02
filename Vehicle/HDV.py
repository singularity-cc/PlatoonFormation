import numpy as np
from Road import *
from Utilities import *
from .Vehicle import *
import random


class HDV(Vehicle):
    """Human drive vehicle class"""

    def __init__(self, dt, simulation, v_des, permanent_id, lane, id, veh_state, veh_param, veh_input=VehicleInput(0, 0)):
        # we need some label to index the HDV lane, id....
        super().__init__(dt, simulation, v_des, permanent_id,
                         lane, id, veh_state, veh_param, veh_input)

        self.category = "HDV"
        self.energy_consumption = 0
        self.car_following_controller = None
        self.lane_change_controller = None
        self.longitudinal_vehicle_plant = None

    def add_car_following_controller(self, car_following_controller):
        self.car_following_controller = car_following_controller

    def add_lane_change_controller(self, lane_change_controller):
        self.lane_change_controller = lane_change_controller

    def add_longitudinal_vehicle_plant(self, longitudinal_vehicle_plant):
        self.longitudinal_vehicle_plant = longitudinal_vehicle_plant

    def update_car_following_control(self):
        if self.car_following_controller is not None:
            self.car_following_controller.update()

    def update_lane_change_control(self):
        if self.lane_change_controller is not None:
            self.lane_change_controller.update()

    def update_vehicle_state(self):
        if self.longitudinal_vehicle_plant is not None:
            self.longitudinal_vehicle_plant.update()

    """******************************************  Interface: First Level Main Functions  **********************************************************"""
    # update vehicle information

    def update(self):
        """Interface Function to update HDV decisions and states"""
        self.update_lane_change_control()
        self.update_car_following_control()
        self.update_vehicle_state()
        self.update_consumption()
        # self.record_state()
        if self.simulation.count % 100 == 0:
            self.update_labels()

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
