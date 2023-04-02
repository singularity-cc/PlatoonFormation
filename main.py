from Visualizer import *
from Road import *
from Vehicle import *
from Simulation import *
from gurobipy import *

"""Road Design 1"""
# road_segments = [
#     Segment(SegmentPoint(100, 700, 0), SegmentPoint(800, 700, 0), 3),
#     Segment(SegmentPoint(800, 700, 0), SegmentPoint(800, 200, 180), 3),
#     Segment(SegmentPoint(800, 200, 180), SegmentPoint(300, 200, 180), 3),
#     Segment(SegmentPoint(300, 200, 180), SegmentPoint(300, 500, 0), 3),
#     Segment(SegmentPoint(300, 500, 0), SegmentPoint(800, 500, 0), 3)
# ]

# waypoints = [
#     Point(100, 700), Point(800, 700), Point(800, 200), Point(300, 200), Point(300, 500), Point(800, 500)
# ]

"""Road Design 2"""
# road_segments = [
#     Segment(Point(100, 700, 0), Point(800, 500, 90), 3),
#     # Segment(Point(800, 700, 0), Point(800, 200, 180), 3),
#     # Segment(Point(800, 200, 180), Point(300, 200, 180), 3),
#     # Segment(Point(300, 200, 180), Point(300, 500, 0), 3),
#     # Segment(Point(300, 500, 0), Point(800, 500, 0), 3)
# ]

# waypoints = [
#     Point(100, 500), Point(500, 300), Point(800, 200), Point(1000, 500), Point(500, 700), Point(700, 500)
# ]

waypoints = [
    Point(10, 10), Point(1000, 10) #, Point(20000, 10)
]


def main():
    # road = Road(waypoints)
    # for segment in road.segments:
    #     print(f"segment_start: {segment.start.x},{segment.start.y},{segment.start.heading}; segment_end: {segment.end.x},{segment.end.y},{segment.end.heading}; \
    #         segment_mid: {segment.mid_arc_point_getter()} \
    #     center: {segment.arc_center.x if segment.arc_center is not None else None}, {segment.arc_center.y if segment.arc_center is not None else None}; radius: {segment.arc_radius}\
    #     arc start angle: {segment.arc_start_angle}, arc end angle: {segment.arc_end_angle}; is_clockwise: {segment.is_clockwise}")
    
    # dt = 0.01

    # hdv = HDV(dt, VehicleState(100, 100, 20, 0), VehicleParameter(10, 4), VehicleInput())
    # traffic = Traffic(road)
    # for i in range(100):
    #     traffic.init_vehicle()
    # print(traffic.vehicles)

    single_cav_stactic_obstacles = 0
    single_cav_dynamic_obstacles = 1
    multi_cavs_no_obstacle = 2
    multi_cavs_dynamic_obstacles = 3
    low_traffic_5CAVs = 4
    mid_traffic_5CAVs = 5
    high_traffic_5CAVs = 6

    sim = Simulation() #road, traffic
    sim.create_road(waypoints, init_heading=0, num_lanes = 3, road_width = 4)
    sim.create_generator(-1)
    sim.create_platoon()
    

    visualizer = Visualizer(sim)
    visualizer.on_execute()

if __name__ == "__main__":
    main()

