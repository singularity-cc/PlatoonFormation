class VehiclePlant:
    def __init__(self, vehicle):
        self.vehicle = vehicle

        """************************************CAV Simulated Physical Plant **********************************************"""
    
    def update_state(self):
        self.update_lateral_state()
        self.update_longitudinal_state()


    def update_lateral_state(self):
        """Vehicle plant simulated by dynamic bicycle model"""
        g=9.81
        m=1500
        Iz=3000
        cf=38000
        cr=66000
        lf=2
        lr=3
        mju=0.02 # friction coefficient

        beta = self.lateral_state.beta
        heading = self.lateral_state.heading
        d_heading = self.lateral_state.heading_dot
        delta = self.input.steer_angle
        v = self.state.v

        # the dynamics
        if v > 1:
            d_beta = -(cf + cr) / (m * v) * beta + ((cr * lr - cf * lf) / (m * v**2) - 1) * d_heading + cf / (m * v) * delta
            dd_heading = - (cr * lr - cf * lf) / Iz * beta - (cr * lr**2 + cf * lf**2) / (Iz * v) * d_heading + cf * lf / Iz * delta
        else:
            d_beta = 0
            dd_heading = 0
        
        self.lateral_state.beta += self.dt * d_beta
        self.lateral_state.heading += self.dt * d_heading
        self.lateral_state.heading_dot += self.dt * dd_heading

        # update state heading
        self.state.heading = self.lateral_state.heading
        heading = self.lateral_state.heading / 180 * np.pi
        beta = self.lateral_state.beta / 180 * np.pi
        # update state positions
        self.state.x += self.dt * (v * np.cos(heading) - v * beta * np.sin(heading))
        self.state.y += self.dt * (v * np.sin(heading) + v * beta * np.cos(heading))

    def update_longitudinal_state(self):
        ca = 0.01
        cr = 0.001
        acc_des = self.input.acc
        v = self.state.v
        m = 1500

        acc = acc_des - (ca * v**2 + cr * v) / m
        self.state.v += self.dt * acc

