import numpy as np
import copy

class Point:
    """Point class"""
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, rhs):
        return Point(self.x + rhs.x, self.y + rhs.y)

    def __sub__(self, rhs):
        return Vector(self.x - rhs.x, self.y - rhs.y)

    def __mul__(self, rhs):
        return Point(self.x * rhs, self.y * rhs)

    def __str__(self):
        return f"{self.x}, {self.y}"

def distance(point1, point2):
    return np.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

class Vector:
    """Vector class"""
    epislon = 0.0000001

    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

    def is_empty(self):
        return self.x == 0 and self.y == 0

    """Linear algebra common operations"""
    def normalized_vector(self):
        point = copy.deepcopy(self)
        x, y = point.x, point.y
        point.x /= np.sqrt(x**2 + y**2)
        point.y /= np.sqrt(x**2 + y**2)
        return point

    def normalize(self):
        x, y = self.x, self.y
        self.x /= np.sqrt(x**2 + y**2)
        self.y /= np.sqrt(x**2 + y**2)
        return self

    def is_parallel(self, rhs):
        return abs(self.cross_product(rhs)) < self.epislon

    def convert_to_radius(self): #
        return np.arctan2(self.y, self.x)

    def convert_to_angle(self): # [-180, 180]
        return self.convert_to_radius() * 180.0 / np.pi

    def find_vertical_line(self):
        line = Line()
        line.init_norm(self)
        return line

    def inner_product(self, rhs):
        return self.x * rhs.x + self.y * rhs.y

    def cross_product(self, rhs):
        return self.x * rhs.y - self.y * rhs.x

    def length(self):
        return np.sqrt(self.x * self.x + self.y * self.y)

    def clockwise_rotate(self, theta):
        self.anti_clockwise_rotate(-theta)

    def anti_clockwise_rotate(self, theta):
        x, y = self.x, self.y
        self.x = np.cos(theta) * x - np.sin(theta) * y
        self.y = np.cos(theta) * y + np.sin(theta) * x

    def deep_copy(self):
        return Vector(self.x, self.y)

    """Operator overloading functions"""

    def multiply(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)

    def __mul__(self, scalar):
        return self.multiply(scalar)

    def __str__(self):
        return f"{self.x}, {self.y}"

def angle_between_vectors(vec1, vec2):
    # to do: cos(theta) = a ' b / (|a| * |b|) 
    return np.arccos(vec1.inner_product(vec2) / (vec1.length() * vec2.length()))

def angle_between_vectors_with_sign(start, end):
    abs_angle = angle_between_vectors(start, end)
    sign = 1 if start.cross_product(end) > 0 else -1
    return sign * abs_angle


class Line: # Form ax + by = c
    """Line class"""
    def __init__(self):
        self.a = None
        self.b = None
        self.c = None

    def __str__(self):
        return f"{self.a} * x + {self.b} * y = {self.c}"

    def init_line(self, head_vec, point = Point(0, 0)):
        self.a = head_vec.y
        self.b = -head_vec.x
        self.c = self.a * point.x + self.b * point.y
        return self

    def init_norm(self, head_vec, point = Point(0, 0)):
        self.a = head_vec.x
        self.b = head_vec.y
        self.c = self.a * point.x + self.b * point.y
        return self

    def projection(self, point: Point) -> Point:
        # projection equation
        D = self.a ** 2 + self.b ** 2
        Dx = self.b ** 2 * point.x - self.a * self.b * point.y + self.a * self.c
        Dy = self.a ** 2 * point.y - self.a * self.b * point.x + self.b * self.c
        return Point(Dx / D, Dy / D)

    def convert_to_vector(self):
        return Vector(-self.b, self.a).normalize()

class LineSegment:
    """Line segment class"""
    def __init__(self, start, end):
        self.start = start
        self.end = end
    
    def is_point_projection_in(self, point):
        vec = Vector(self.end.x - self.start.x, self.end.y - self.start.y)
        vec1 = Vector(point.x - self.start.x, point.y - self.start.y)
        vec2 = Vector(point.y - self.end.y, point.y - self.end.y)

        sign1 = vec.inner_product(vec1)
        sign2 = vec.inner_product(vec2)
        return sign1 * sign2 <= 0


def intersect(line1: Line, line2: Line) -> Point:
    D = line1.a * line2.b - line1.b * line2.a
    if D == 0:
        # line1 and line2 are parallel
        return None

    # cramer's rule
    Dx = line1.c * line2.b - line1.b * line2.c
    Dy = line1.a * line2.c - line2.a * line1.c
    return Point(Dx / D, Dy / D)