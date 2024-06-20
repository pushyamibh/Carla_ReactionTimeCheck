import sys
import glob
import os

#The added path depends on where the carla binaries are stored
try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time
import math
import numpy as np
import random
from agents.navigation.controller import VehiclePIDController

VEHICLE_VEL = 5
class Player():
    def __init__(self, world, bp, vel_ref = VEHICLE_VEL, max_throt = 0.75, max_brake = 0.3, max_steer = 0.8):
        self.world = world
        self.max_throt = max_throt
        self.max_brake = max_brake
        self.max_steer = max_steer
        self.vehicle = None
        self.bp = bp 
        while(self.vehicle is None):
            spawn_points = world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        
        self.spectator = world.get_spectator()
        
        dt = 1.0 / 20.0
        args_lateral_dict = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': dt}
        args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': dt}
        offset = 0
        
        self.controller = VehiclePIDController(self.vehicle,
                                        args_lateral=args_lateral_dict,
                                        args_longitudinal=args_longitudinal_dict,
                                        offset=offset,
                                        max_throttle=max_throt,
                                        max_brake=max_brake,
                                        max_steering=max_steer)
        self.vel_ref = vel_ref
        self.waypointsList = []
        self.current_pos = self.vehicle.get_transform().location
        self.past_pos = self.vehicle.get_transform().location
    
    def dist2Waypoint(self, waypoint):
        vehicle_transform = self.vehicle.get_transform()
        vehicle_x = vehicle_transform.location.x
        vehicle_y = vehicle_transform.location.y
        waypoint_x = waypoint.transform.location.x
        waypoint_y = waypoint.transform.location.y
        return math.sqrt((vehicle_x - waypoint_x)**2 + (vehicle_y - waypoint_y)**2)
    
    def go2Waypoint(self, waypoint, draw_waypoint = True, threshold = 0.3):
        if draw_waypoint :
            # print(" I draw") 
            self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                                       color=carla.Color(r=255, g=0, b=0), life_time=10.0,
                                                       persistent_lines=True)
        
        current_pos_np = np.array([self.current_pos.x,self.current_pos.y])
        past_pos_np = np.array([self.past_pos.x,self.past_pos.y])
        waypoint_np = np.array([waypoint.transform.location.x,waypoint.transform.location.y])
        vec2wp = waypoint_np - current_pos_np
        motion_vec = current_pos_np - past_pos_np
        dot = np.dot(vec2wp,motion_vec)
        if (dot >=0):
            while(self.dist2Waypoint(waypoint) > threshold) :
        
                control_signal = self.controller.run_step(self.vel_ref,waypoint)
                    
                self.vehicle.apply_control(control_signal)
                
                self.update_spectator()

    def getLeftLaneWaypoints(self, offset = 2*VEHICLE_VEL, separation = 0.3):
        current_waypoint = self.world.get_map().get_waypoint(self.vehicle.get_location())
        left_lane = current_waypoint.get_left_lane()
        self.waypointsList = left_lane.previous(offset)[0].previous_until_lane_start(separation)

    def getRightLaneWaypoints(self, offset = 2*VEHICLE_VEL, separation = 0.3):
        current_waypoint = self.world.get_map().get_waypoint(self.vehicle.get_location())
        right_lane = current_waypoint.get_left_lane()
        self.waypointsList = right_lane.next(offset)[0].next_until_lane_end(separation)
    
    def do_left_lane_change(self):
        
        self.getLeftLaneWaypoints()
        for i in range(len(self.waypointsList)-1):
            self.current_pos = self.vehicle.get_location()
            self.go2Waypoint(self.waypointsList[i])
            self.past_pos = self.current_pos
            self.update_spectator()

    def do_right_lane_change(self):

        self.getRightLaneWaypoints()
        for i in range(len(self.waypointsList)-1):
            self.current_pos = self.vehicle.get_location()
            self.go2Waypoint(self.waypointsList[i])
            self.past_pos = self.current_pos
            self.update_spectator()

    def update_spectator(self):
        new_yaw = math.radians(self.vehicle.get_transform().rotation.yaw)
        spectator_transform =  self.vehicle.get_transform()
        spectator_transform.location += carla.Location(x = -10*math.cos(new_yaw), y= -10*math.sin(new_yaw), z = 5.0)
        
        self.spectator.set_transform(spectator_transform)
        self.world.tick()

    def is_waypoint_in_direction_of_motion(self,waypoint):
        current_pos = self.vehicle.get_location()

    def draw_waypoints(self):
        for waypoint in self.waypointsList:
            self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                                       color=carla.Color(r=255, g=0, b=0), life_time=10.0,
                                                       persistent_lines=True)

dt = 1.0 / 20.0
args_lateral_dict = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': dt}
args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': dt}

offset = 0
VEHICLE_VEL = 10

actorList = []
try:
    client = carla.Client("localhost",2000)
    client.set_timeout(10.0)
    world = client.load_world("Town02")

    spectator = world.get_spectator()
    actorList.append(spectator)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1/20
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("cybertruck")[0]
    
    player = Player(world, vehicle_bp)
    actorList.append(player.vehicle)
    actorList.append(player.spectator)

    while(True):
        player.update_spectator()
        manoeuver = input("Enter manoeuver: ")
        if ( manoeuver == "l"): #Perform left lane change
            player.do_left_lane_change()
        elif (manoeuver == "r"): #Perform right lane change
            player.do_right_lane_change()
        elif (manoeuver == "d"): #Just draws the waypoints for debugging purpose
            player.getLeftLaneWaypoints()
            player.draw_waypoints()
    
finally:
    print("Destroying actors")
    client.apply_batch([carla.command.DestroyActor(x) for x in actorList])