import numpy as np
import math
# import matplotlib as plt
import pygame
from pygame import gfxdraw
from Road import *
# from Assets import *
from Simulation import *

class Visualizer:
    """Simulation visualizer implemented using Pygame"""
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    LIGHTBLUE = (180, 180, 220)
    BLUE = (0, 0, 255)

    def __init__(self, simulator = None): #, road_simulator = None, HDV_simulator = None, CAV_simulator = None
        """Initialize necessary variables for pygame framework"""
        self.running = True
        self.window = None
        self.size = self.width, self.height = 1200, 900
        self.zoom = 1
        self.offset = (0, 0)
        self.mouse_last = (0, 0)
        self.mouse_down = False
        self.camera_on = False
        self.dt = simulator.dt
        self.fps = int(1.0 / self.dt)

        """Initialize simulator including road simulator, vehicle generator"""
        self.simulator = simulator
        self.road_simulator = self.simulator.road # road_simulator
        self.generator = self.simulator.generator

        # self.HDV = pygame.image.load("green_car.png")
        # self.CAV = pygame.image.load("red_car.png")

    def on_init(self):
        """Initialize the pygame"""
        pygame.init()
        self.window = pygame.display.set_mode(self.size) #, pygame.HWSURFACE | pygame.DOUBLEBUF
        self.running = True
        self.clock = pygame.time.Clock()
        pygame.font.init()
        self.text_font = pygame.font.SysFont('Lucida Console', 16)
        pygame.display.update()

    def on_execute(self):
        """Execute the visualization at each simulation step"""
        # initialize the visualization
        self.on_init()

        # update during the visualized simulation
        while self.running:
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
        
        # clean up after the visualized simulation ends
        self.on_cleanup()

    def on_cleanup(self):
        pygame.quit()

    def on_event(self, event):
        """Listen to events during the visualization, including quit, keyborad, mouse events"""
        # Handle quit events
        if event.type == pygame.QUIT:
            self.running = False

        # Handle keyborard events
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_s:
                pygame.image.save(self.window, "screenshot.jpg")
            elif event.key == pygame.K_r:
                self.simulator.output_record_data_as_excel()
            elif event.key == pygame.K_c:
                if self.camera_on == False and self.simulator.platoon.cavs:
                    self.camera_on = True
                else:
                    self.camera_on = False


        # Handle mouse events
        elif event.type == pygame.MOUSEBUTTONDOWN:
            # If mouse button down
            if event.button == 1:
                # Left click
                x, y = pygame.mouse.get_pos()
                x0, y0 = self.offset
                self.mouse_last = (x-x0*self.zoom, y-y0*self.zoom)
                self.mouse_down = True
            if event.button == 4:
                # Mouse wheel up
                self.zoom *=  (self.zoom**2+self.zoom/4+1) / (self.zoom**2+1)
            if event.button == 5:
                # Mouse wheel down 
                self.zoom *= (self.zoom**2+1) / (self.zoom**2+self.zoom/4+1)
        elif event.type == pygame.MOUSEMOTION:
            # Drag content
            if self.mouse_down:
                x1, y1 = self.mouse_last
                x2, y2 = pygame.mouse.get_pos()
                self.offset = ((x2-x1)/self.zoom, (y2-y1)/self.zoom)
        elif event.type == pygame.MOUSEBUTTONUP:
            self.mouse_down = False  
    
    def on_loop(self):
        """Game Loop: things to do at every simulation step"""

        self.window.fill(self.WHITE)
        """Update the simulation"""
        self.simulator.update()

        # update camera offset
        # TODO: Refactor the code
        if self.camera_on:
            # camera offset
            camera = self.simulator.platoon.cavs[0].point_location()
            # camera = (head_cav_pos.x, head_cav_pos.y)
            self.offset = (-camera.x, -camera.y)


        """Draw the updated simulation"""
        self.draw()
        """Display the updated simulation"""
        pygame.display.update()

        # update the time tick
        self.clock.tick(self.fps)

    """****************************************** Draw Functions  **********************************************************"""
    def draw(self):
        """Draw function to call drawing road, traffic, state"""
        self.draw_road()
        self.draw_traffic()
        self.draw_status()

    def draw_road(self):
        """Draw road function: draw road structure"""
        road_segments = self.road_simulator.segments
        for road_segment in road_segments:
            self.draw_road_segement_color(road_segment)
            self.draw_road_segment_skeleton(road_segment)


    def draw_traffic(self):
        """Draw traffic function: draw all the HDVs on road"""
        for segment in self.road_simulator.segments:
            self.draw_vehicles_in_segment(segment)

    def draw_status(self):
        """Draw visualization states"""
        text_fps = self.text_font.render(f't={self.simulator.t:.5}', False, (0, 0, 0))
        text_frc = self.text_font.render(f'n={self.simulator.count}', False, (0, 0, 0))
        text_energy = self.text_font.render(f'energy={round(self.simulator.energy_consumption,2)}', False, (0, 0, 0))
        
        self.window.blit(text_fps, (0, 0))
        self.window.blit(text_frc, (100, 0))
        # self.window.blit(text_energy, (0, 100))

    """Draw functions"""

    def draw_vehicles_in_segment(self, segment):
        """Draw all vehicles in one segment function"""
        for vehicles_in_lane in segment.vehicles:
            for vehicle in vehicles_in_lane:
                if vehicle.category == "CAV":
                    # self.draw_reference_line(vehicle, self.RED)
                    # self.draw_path_samples(vehicle, self.BLUE)
                    self.draw_target(vehicle, self.BLUE)
                    self.draw_best_trajectry(vehicle, self.RED)

        for vehicles_in_lane in segment.vehicles:
            for vehicle in vehicles_in_lane:
                if vehicle.category == "HDV":
                    self.draw_HDV(vehicle)
                else:
                    self.draw_CAV(vehicle)       

    # def draw_HDVs_in_segment(self, segment):
    #     """Draw all HDVs in one segment function"""
    #     for vehicles_in_lane in segment.vehicles:
    #         for vehicle in vehicles_in_lane:
    #             self.draw_HDV(vehicle)

    def draw_CAV(self, cav):
        """Draw one CAV function"""
        # self.draw_reference_line(cav, self.GREEN)
        self.draw_vehicle(cav, self.RED)
        
        

    def draw_HDV(self, hdv):
        """Draw one HDV function"""
        self.draw_vehicle(hdv, self.BLUE)


    def draw_vehicle(self, veh, color):
        """Draw a vehicle"""
        state = veh.state
        param = veh.param
        pos = (state.x, state.y)
        size = (param.length, param.width)
        angle = np.pi * state.heading / 180.0

        self.rotated_box(pos, size, angle, color=color)   

    ##### Draw CAV target and motion trajectory
    def draw_target(self, cav, color = BLUE):
        if cav.is_platooning == 2:
            return

        self.draw_circle(cav.target_location.x, cav.target_location.y, 1, True, color)

    def draw_best_trajectry(self, cav, color = RED):
        if cav.is_platooning == 2:
            return 

        for point, speed in cav.best_trajectory_cartessian:
            self.draw_circle(point.x, point.y, 0.5, True, color)

    def draw_reference_line(self, cav, color = BLACK):
        if cav.is_platooning == 2:
            return

        for point in cav.discrete_path_reference:
           self.draw_circle(point.x, point.y, 0.2, True, color)

        look_ahead_point = cav.look_ahead_point
        self.draw_circle(look_ahead_point.x, look_ahead_point.y, 0.5, True, self.BLUE)

    def draw_path_samples(self, cav, color = BLACK):
        if cav.is_platooning == 2:
            return
        # for point in cav.discrete_path_reference:
        #     self.draw_circle(point.x, point.y, 1, True, color)
        for point in cav.visualization_set:
            self.draw_circle(point.x, point.y, 1, True, color)
        
        # for point in cav.dense_visualization_set:
        #     self.draw_circle(point.x, point.y, 1, True, self.BLACK)

        for point in cav.dense_visualization_set:
            self.draw_circle(point.x, point.y, 0.6, True, self.BLACK)

        

    """****************************************** Support Functions for Draw Road Functions  **********************************************************"""

    def draw_road_segement_color(self, road_segment, color = LIGHTBLUE):
        """Todo: Draw the color of road segment"""
        pass

    def draw_road_segment_skeleton(self, road_segment, color = BLACK):
        """Draw the skeleton of the road segment"""
        mid = road_segment.num_lanes // 2
        if road_segment.num_lanes % 2 == 1:
            # if the number of lanes is odd
            self.draw_road_segment_lane(road_segment, 0.5)
            self.draw_road_segment_lane(road_segment, -0.5)
            for i in range(1, mid + 1):
                self.draw_road_segment_lane(road_segment, i + 0.5)
                self.draw_road_segment_lane(road_segment, -i - 0.5)
        else:
            # if the number of lanes is even
            self.draw_road_segment_lane(road_segment, 0)
            for i in range(1, mid + 1):
                self.draw_road_segment_lane(road_segment, i)
                self.draw_road_segment_lane(road_segment, -i)

    def draw_road_segment_lane(self, road_segment, lane_width_factor):
        """Draw all lanes of one road segment"""
        if road_segment.start.heading == road_segment.end.heading or road_segment.arc_center.x == None or road_segment.arc_center.y == None:
            self.draw_lane_line(road_segment, lane_width_factor)
        else:
            self.draw_lane_arc(road_segment, lane_width_factor) 

    def draw_lane_line(self, road_segment, lane_width_factor):
        """Draw one straight lane using line"""
        # extract information from road_segment
        lane_width = self.road_simulator.lane_width
        start_angle = road_segment.start.heading / 180 * np.pi
        end_angle = road_segment.end.heading / 180 * np.pi
        start_cos, start_sin = np.cos(start_angle), np.sin(start_angle)
        end_cos, end_sin = np.cos(end_angle), np.sin(end_angle)
        start_lane_dx, start_lane_dy = lane_width * start_sin, lane_width * start_cos
        end_lane_dx, end_lane_dy = lane_width * end_sin, lane_width * end_cos

        # draw straight line
        self.draw_line((road_segment.start.x + lane_width_factor * start_lane_dx), (road_segment.start.y - lane_width_factor * start_lane_dy), 
            (road_segment.end.x + lane_width_factor * end_lane_dx), (road_segment.end.y - lane_width_factor * end_lane_dy), self.BLACK)


    def draw_lane_arc(self, road_segment, lane_width_factor):
        """Draw one curved lane using arc"""
        # extract information from road_segment
        lane_width = self.road_simulator.lane_width
        radius, center = road_segment.arc_radius, road_segment.arc_center
        arc_start_angle, arc_end_angle, is_clockwise = road_segment.arc_start_angle, road_segment.arc_end_angle, road_segment.is_clockwise

        # draw arc 
        if is_clockwise:
            self.draw_arc(center.x, center.y, int(radius + lane_width_factor * lane_width), math.floor(arc_start_angle), math.ceil(arc_end_angle), self.BLACK)
        else:
            self.draw_arc(center.x, center.y, int(radius + lane_width_factor * lane_width), math.floor(arc_end_angle), math.ceil(arc_start_angle), self.BLACK)



    """****************************************** Support Functions to Draw Basic Shapes  **********************************************************"""

    def rotated_box(self, pos, size, angle=None, cos=0, sin=0, centered=True, color=(0, 0, 255), filled=True):# angle is the numerical angle
        """Draws a rectangle center at *pos* with size *size* rotated anti-clockwise by *angle*."""
        x, y = pos
        l, h = size

        cos, sin = np.cos(angle), np.sin(angle)
        
        vertex = lambda e1, e2: (
            x + (e1*l*cos + e2*h*sin)/2,
            y + (e1*l*sin - e2*h*cos)/2
        )

        if centered:
            vertices = self.convert(
                [vertex(*e) for e in [(-1,-1), (-1, 1), (1,1), (1,-1)]]
            )
        else:
            vertices = self.convert(
                [vertex(*e) for e in [(0,-1), (0, 1), (2,1), (2,-1)]]
            )

        self.polygon(vertices, color, filled=filled)

    def polygon(self, vertices, color, filled=True):
        """Draw polygon: either filled with color or not"""
        gfxdraw.aapolygon(self.window, vertices, color)
        if filled:
            gfxdraw.filled_polygon(self.window, vertices, color)

    """Draw Vehicle Support Functions"""
    def draw_rotated_box(self, vertices, color = RED):
        """Draw rotated box function"""
        gfxdraw.filled_polygon(self.window, self.convert(vertices), color)
        
    def draw_line(self, start_x, start_y, end_x, end_y, color = BLACK):
        """Draw a line"""
        gfxdraw.line(self.window, *self.convert(start_x, start_y), 
            *self.convert(end_x, end_y), color)       
    
    def draw_arc(self, center_x, center_y, radius, start_angle, end_angle, color = BLACK):
        """Draw an arc"""
        gfxdraw.arc(self.window, *self.convert(center_x, center_y), int(radius * self.zoom), start_angle, end_angle, color)
    
    def draw_circle(self, center_x, center_y, radius, filled = True, color = BLACK):
        if filled is True:
            gfxdraw.filled_circle(self.window, *self.convert(center_x, center_y), int(radius * self.zoom), color)
        else:
            gfxdraw.circle(self.window, *self.convert(center_x, center_y), int(radius * self.zoom), color)

    def convert(self, x, y=None):
        """Convert function to enable zoom in / zoom out"""


        if isinstance(x, list) or isinstance(x, tuple):
            return [self.convert(e[0], e[1]) for e in x]

        # print(f"convert function, x: {x}, y: {y}; offset: ({self.offset[0]},{self.offset[1]}); zoom: {self.zoom}")
        return (
            int(self.width/2+(x + self.offset[0])*self.zoom), 
            int(self.height/2+(y + self.offset[1])*self.zoom),       
        )























