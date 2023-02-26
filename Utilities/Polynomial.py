
class MotionCurve:
    def __init__(self):
        self.coefficients = []
    
    def compute(self, order, length):
        return 0

class MotionState1D:
    def __init__(self, f, df, ddf):
        self.f = f
        self.df = df
        self.ddf = ddf


class QuinticPolynomial:
    def __init__(self, start_motion_state, end_motion_state, p_length):
        self.start_motion_state = start_motion_state
        self.end_motion_state = end_motion_state
        self.p_length = p_length
        self.coeff = [0, 0, 0, 0, 0, 0]

        self.update_coeff()
        # print("quintic polynomial is generated!")

    def __str__(self):
        return f"{self.coeff[0]},{self.coeff[1]},{self.coeff[2]},{self.coeff[3]},{self.coeff[4]},{self.coeff[5]}"

    def duration(self):
        return self.p_length

    def compute(self, order, length):
        if order == 0:
            return self.compute_s(length)
        elif order == 1:
            return self.compute_v(length)
        elif order == 2:
            return self.compute_a(length)
        return 0.0

    def compute_s(self, length):
        return self.coeff[0] + length * (self.coeff[1] + length * (self.coeff[2] + length * (self.coeff[3] + length * (self.coeff[4] + length * self.coeff[5]))))

    def compute_v(self, length):
        return self.coeff[1] + length * (2 * self.coeff[2] + length * (3 * self.coeff[3] + length * (4 * self.coeff[4] + length * 5 * self.coeff[5])))

    def compute_a(self, length):
        return 2 * self.coeff[2] + length * (6 * self.coeff[3] + length * (12 * self.coeff[4] + length * 20 * self.coeff[5]))
    
    def update_coeff(self):
        self.coeff[0] = self.start_motion_state.f
        self.coeff[1] = self.start_motion_state.df
        self.coeff[2] = self.start_motion_state.ddf * 0.5

        p1 = self.p_length
        p2 = p1 * p1
        p3 = p2 * p1
        c0 = (self.end_motion_state.f - self.coeff[0] - self.coeff[1] * p1 - self.coeff[2] * p2) / p3
        c1 = (self.end_motion_state.df - self.coeff[1] - 2 * self.coeff[2] * p1) / p2
        c2 = (self.end_motion_state.ddf - 2 * self.coeff[2]) / p1

        self.coeff[3] = 0.5 * (c2 + 20 * c0 - 8 * c1)
        self.coeff[4] = (-15 * c0 + 7 * c1 - c2) / p1
        self.coeff[5] = (c2 * 0.5 + 6 * c0 - 3 * c1) /p2





class QuarticPolynomial:
    def __init__(self, start_motion_state, end_motion_state):
        self.start_motion_state = start_motion_state
        self.end_motion_state = end_motion_state
        self.coeff = [0, 0, 0, 0, 0]

        self.update_coeff()

    def compute(self, order, length):
        if order == 0:
            return self.compute_s(length)
        elif order == 1:
            return self.compute_v(length)
        elif order == 2:
            return self.compute_a(length)
        return 0.0
    
    def compute_s(self, length):
        return self.coeff[0] + length * (self.coeff[1] + length * (self.coeff[2] + length * (self.coeff[3] + length * self.coeff[4])))

    def compute_v(self, length):
        return self.coeff[1] + length * (2 * self.coeff[2] + length * (3 * self.coeff[3] + length * 4 * self.coeff[4]))

    def compute_a(self, length):
        return 2 * self.coeff[2] + length * (6 * self.coeff[3] + length * 12 * self.coeff[4])

    def update_coeff(self):
        self.coeff[0] = self.start_motion_state.f
        self.coeff[1] = self.start_motion_state.df
        self.coeff[2] = self.start_motion_state.ddf / 2.0

        # TODO: update other coeff


# if __name__ == "__main__":
#     # Test
#     start_motion_state = MotionState1D(0, 0, 0)
#     end_motion_state = MotionState1D(10, 0, 0)
#     p_length = 1
#     quintic_poly = QuinticPolynomial(start_motion_state, end_motion_state, p_length)
#     print(quintic_poly)
#     print(f"Quintic end is {quintic_poly.compute(0, p_length)}")
#     print(f"Quintic mid is {quintic_poly.compute(0, p_length / 2.0)}")


#     start_motion_state = MotionState1D(4, 0, 0)
#     end_motion_state = MotionState1D(0, 0, 0)
#     p_length = 37.5
#     quintic_poly = QuinticPolynomial(start_motion_state, end_motion_state, p_length)
    
#     print(quintic_poly)
#     print(f"Quintic end is {quintic_poly.compute(0, p_length)}")
#     print(f"Quintic at 15.0 is {quintic_poly.compute(0, 15.0)}")

    