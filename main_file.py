import carla
from carla import ColorConverter as cc
import random
import time
import cv2
import numpy as np
import math
import pygame
from ruamel.yaml import YAML
from pathlib import Path
from shared_memory_dict import SharedMemoryDict
import os
import numpy as np
import json
#import evdev
#from evdev import ecodes, InputDevice, ff
from carla import Sensor, Actor

from pygame.locals import K_a
from pygame.locals import K_w
from pygame.locals import K_d
from pygame.locals import K_x
from pygame.locals import K_f
from pygame.locals import K_t
from pygame.locals import K_h
from pygame.locals import K_b
from pygame.locals import K_p
from pygame.locals import K_s
from pygame.locals import K_UP
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_DOWN
# Define shared memory object and size in bytes
smd = SharedMemoryDict(name='tokens', size=10000000)

# Read Config File
configfile=Path("config.yaml")
_config = YAML(typ='safe').load(configfile)

control_device=_config['sim']['default_control']

print("Please enter the user ID : ")
input_user=input()
print("Thank you!, You may continue the game and make sure the camera is ON!!")
time.sleep(5)

class mirror_parameters:
    def __init__(self):
        self.left_yaw=-150
        self.left_pitch=0
        self.right_yaw=150
        self.right_pitch=0


# Render object to keep and pass the PyGame surface
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))
        #self.display=display


# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))

# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def process_image_data(image_data, view_id, flip=False):
    img = np.reshape(np.copy(image_data.raw_data), (image_data.height, image_data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)	
    if flip:
        img=cv2.flip(img, 1)
    smd[view_id] = img



class FadingText(object):
    def __init__(self, font, dim, pos):
        
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.clock=pygame.time.Clock()

    def set_text(self, text, seconds, color=(255, 255, 255)):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        
       # self.surface.set_alpha(1000.0 * self.seconds_left)
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))
        

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)
'''
class GetActorsFromWorld(object):
  
    #returns all properties of carla world.
    
    def __init__(self, world):
        
        #initilize the carla client.
       
        #self.carla_client = carla_client
        #self.carla_logger = logging.getLogger('carla_client_logger')
        #self.carla_logger.info("Carla Server initialized....")
        self.world = world
        self.actors_list = None
        self.actor_dict = dict()
    
    def get_dict_of_actors(self):
        
        #return available dict of actors obj.
        
        #self.world = self.world.get_world()                
        self.actors_list = self.world.get_actors()
        self.actor_dict["vehicle"] = self.actors_list.filter('vehicle.*')
        self.actor_dict["traffic"] = self.actors_list.filter('traffic.*')        
        print(self.actors_list)
        #return self.actor_dict
'''
class controller ():
    def __init__(self,world, vehicle,display):
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        self.autopilot=False
        self.info_text = []
        self.display = display
        self.autopilot_stop_time = None
        self.brake_reaction_time = 0.0
        self.brake_start_time = None
        self.flag_1=True
        self.flag_2=True
        self.steering_reaction_time = 0.0
        self.velocity_history = []  # Store vehicle velocities during TOR to obstacle
        self.max_longitudinal_deceleration = []
        self.longitudinal_deceleration=[]
        self.dict_time_speed={}
        self.flag_3=True
        self.lateral_positions = []
        self.world=world
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        width=430 #1024
        height= 768 #320
        dim = (width, height)
        self._notifications=FadingText(font, (width, 40), (50, height - 100))
        self.take_over_indication=None
        self.dict_file={}
        self.dict_file['collision']= None
        self.dict_file['take_over_indication']=None
        self.dict_file['brake_reaction_time']=None
        self.dict_file['Logitudinal_deceleration']=None
        self.dict_file['max_longitudinal_deceleration']=None
        self.dict_file['reverse_pressed']=False
        self.dict_file['steering_reaction_time']=None
        self.brake_reaction_time=None
        try:

            # initialize steering wheel
            pygame.joystick.init()

            joystick_count = pygame.joystick.get_count()
            #if joystick_count > 1:
            #    raise ValueError("Please Connect one Joystick")
            if joystick_count ==0 :
                raise ValueError("No joystick connected")

            if control_device=='fanatec':
                self._joystick = pygame.joystick.Joystick(1)
            else:
                self._joystick = pygame.joystick.Joystick(0)

            self._joystick.init()

            self._steer_idx = _config['sim']['controls'][control_device]['steering_wheel']
            self._throttle_idx = _config['sim']['controls'][control_device]['throttle']
            self._brake_idx = _config['sim']['controls'][control_device]['brake']
            self._reverse_idx = _config['sim']['controls'][control_device]['reverse']
            self._handbrake_idx = _config['sim']['controls'][control_device]['handbrake']
            self.vehicle=vehicle
            
            '''
            spectator_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
            spectator_location = vehicle.get_location() - carla.Location(x=-4, y=2,z=0.2)  # Adjust the offset as needed
            spectator_transform = carla.Transform(spectator_location, vehicle.get_transform().rotation)
            #spectator_transform=carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=1)),vehicle.get_transform().rotation)
            spectator_car = self.world.spawn_actor(spectator_bp, spectator_transform)
            vehicle_list.append(spectator_car)
            update_spectator_camera(vehicle, spectator_car)
            '''
        except Exception as e: 
            print(e)
            print('Shutting Down')
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
            world.apply_settings(original_settings)
            pygame.quit()
            print ("Done")
            exit (0)

    
        

    def parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        
        jsButtons = [float(self._joystick.get_button(i)) for i in
                        range(self._joystick.get_numbuttons())]
        
        # Invert Control Signals if needed
        if control_device=='fanatec':
            ic=-1
        else:
            ic=1

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 0.55  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])
        steer_dead_zone=_config['carla']['steering_dead_zone']
        if (steerCmd>=-1*steer_dead_zone and steerCmd<=steer_dead_zone):
            steerCmd=0

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * ic * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd > 0:
            throttleCmd = 0
        elif throttleCmd <= 1:
            throttleCmd = 1
        throttleCmd=((1-throttleCmd))

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * ic * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd > 0:
            brakeCmd = 0
        elif brakeCmd <= 1:
            brakeCmd = 1
        brakeCmd=(abs(1-brakeCmd))
        
        
        self._control.steer = steerCmd
        
        self._control.brake = brakeCmd
        #if (self._control.brake>=0.1):
         #   print("this is control brake", self._control.brake)
        self._control.throttle = throttleCmd

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])
        #print("this is control brake", self._control.throttle)
        #self.vehicle.apply_control(self._control)
        if self._control.throttle > 0:
           # throttle_sound.play()
            channel = pygame.mixer.Channel(0)
            channel.play(throttle_sound)
        else:
            # You may want to adjust the conditions for playing the engine sound
            engine_sound.play()
        '''
        elif self._control.brake > 0:
            brake_sound.play()
            channel_1 = pygame.mixer.Channel(1)
            channel_1.play(brake_sound)
        elif self._control.reverse:
            reverse_sound.play()
            channel_1 = pygame.mixer.Channel(1)
            channel_1.play(reverse_sound)
        '''   

        acc = vehicle.get_acceleration()
       # current_velocity =(3.6 *  math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))

        # Store the velocity during TOR to obstacle
        if self.take_over_indication is not None :
            self.dict_time_speed[time.time()- self.take_over_indication]=-(acc.x)
       



        if self._control.brake >= 0.6 and self.take_over_indication is not None and self.flag_1:
            self.brake_reaction_time =  time.time()-self.take_over_indication 
            self.dict_file['brake_reaction_time']=self.brake_reaction_time
            self.flag_1=False

        if self.take_over_indication is not None :
            if (time.time()- self.take_over_indication)<9.0:
                self.compute_max_longitudinal_deceleration()

        
        if self.take_over_indication is not None:
            #print("This is auto time", self.take_over_indication +8.0," Present time",time.time()*0.02)
            if ((self.take_over_indication +8.0) <= (time.time()) <= (self.take_over_indication+ 18.0)):
                curr_loc=self.vehicle.get_location()
                #print(curr_loc.y)
                self.lateral_positions.append(curr_loc.y) 
            if (self.take_over_indication+ 18.0)<=(time.time())<=(self.take_over_indication+19.0):
                #print(time.time(),self.take_over_indication)
                self.calculate_lateral_position_stddev(self.lateral_positions)

        #if self._control.throttle <= 0.5 and self.brake_start_time is not None:
            #   self.brake_reaction_time =self.autopilot_stop_time - self.brake_start_time
            #print(self.brake_reaction_time)
        
        if (self._control.steer>0.06 or self._control.steer<-0.06) and self.take_over_indication is not None and self.flag_2:
            self.steering_reaction_time = time.time()-self.take_over_indication
            self.dict_file['autopilot_mode']=self.autopilot
            self.dict_file['steering_reaction_time']=self.steering_reaction_time
            self.flag_2=False
        self.vehicle.apply_control(self._control)

    def compute_max_longitudinal_deceleration(self):
        
            
        if self.dict_time_speed:
            # Calculate the deceleration for each velocity point
            decelerations = [0.0]
            for i in (self.dict_time_speed.keys()):
                
                #delta_time = time.time() - self.autopilot_stop_time
                delta_time= i
                
                if delta_time <9:
                    deceleration = self.dict_time_speed[i]
                    decelerations.append(deceleration)
            self.longitudinal_deceleration=decelerations
            self.dict_file['Logitudinal_deceleration']=self.longitudinal_deceleration
            
            # Find the maximum deceleration
            self.max_longitudinal_deceleration = max(decelerations)
            self.dict_file['max_longitudinal_deceleration']=self.max_longitudinal_deceleration

    def calculate_lateral_position_stddev(self,lateral_position):
        
            # Get the lateral positions during the specified time frame
            #lateral_positions = (lateral_position)
            
            #print(lateral_positions)
            if lateral_position:
                # Calculate the standard deviation
                lateral_stddev = np.std(lateral_position)
                self.dict_file['lateral_stddev']=lateral_stddev
                #return lateral_stddev
        

    def toggle_autopilot(self,tm):
        flag=True
        if self.autopilot and flag:
            # Autopilot is being turned off
            self.autopilot_stop_time = time.time()
            flag=False
        else:
            # Autopilot is being turned on
            self.autopilot_stop_time = None
            self.brake_start_time = None
            self.brake_reaction_time = 0.0
            tm.global_percentage_speed_difference(-240)
        self.autopilot = not self.autopilot
        
        self.vehicle.set_autopilot(self.autopilot)
        

    def update_info_text(self, text):
        self.info_text = text
    

    
    def render_text(self, text, position, font, color=(255, 255, 255)):
           
           _font_mono = pygame.font.Font(mono, 15 if os.name == 'nt' else 18)
           v_offset = 4
           if isinstance(text, tuple):
               if isinstance(text[1], bool):
                            #but_text=_font_mono.render(('ON' if text[1] else 'OFF'),True,'red')
                            #rect = pygame.Rect((bar_h_offset, v_offset + 8), (30, 35))
                            pygame.draw.circle(self.display, 'green' if text[1] else 'black', (position[0]+180,position[1]+30), 25)
                            #self.display.blit(but_text,(position[0]+180,position[1]+30)) 
               text = text[0]
          
           if text=="location":
                    channel_2 = pygame.mixer.Channel(2)
                    channel_2.play(warning_sound)
                    #pygame.mixer.Sound.play(warning_sound)
                    
                    self._notifications.set_text("Take Over The Control", seconds=25.0)
                    #self._notifications.tick(self.world,pygame.time.Clock())
                    self._notifications.render(self.display)
                    
                    pygame.mixer.music.stop()
                    self.take_over_indication=time.time()
                    
                    self.dict_file['take_over_indication']= self.take_over_indication
                    self.flag_1=True
                    self.flag_2=True
           if text:
                text_surface = font.render(text, True, color)
                self.display.blit(text_surface, (position[0],position[1])) 

           
           '''
           
           if self.take_over_indication:
                takeover_reaction_time =  self.take_over_indication
                takeover_reaction_text = 'Takeover Reaction Time: {:.2f} seconds'.format(takeover_reaction_time)
                takeover_reaction_surface = font.render(takeover_reaction_text, True, color)
                self.display.blit(takeover_reaction_surface, (30, 60))

           if self.brake_reaction_time > 0:
                brake_reaction_text = 'Brake Reaction Time: {:.2f} seconds'.format(self.brake_reaction_time)
                brake_reaction_surface = font.render(brake_reaction_text, True, color)
                self.display.blit(brake_reaction_surface, (30, 80))
           if self.steering_reaction_time > 0:
                steering_reaction_text = 'Steering Reaction Time: {:.2f} seconds'.format(self.steering_reaction_time)
                steering_reaction_surface = font.render(steering_reaction_text, True, color)
                self.display.blit(steering_reaction_surface, (30, 100))  # Adjust the position as needed
       
           if isinstance(self.max_longitudinal_deceleration, list):
                    if len(self.max_longitudinal_deceleration) > 1:
                        points = [(x + 8,  v_offset +8 + (10.0 - y) * 30) for x, y in enumerate(self.max_longitudinal_deceleration)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
           '''
           pygame.display.flip()

def update_spectator_camera(vehicle, spectator_vehicle):
    # Get the main car's location and transform
    vehicle_location = vehicle.get_location()
    vehicle_transform = vehicle.get_transform()

    # Define an offset for the camera (adjust as needed)
    camera_offset = carla.Location(x=-5, y=0, z=5)

    # Calculate the spectator camera's new location and rotation
    new_location = vehicle_location + camera_offset
    new_rotation = vehicle_transform.rotation

    # Set the spectator camera's new location and rotation
    spectator.set_transform(carla.Transform(new_location, new_rotation))

    # Set the spectator vehicle's location and rotation to follow the main car
    spectator_vehicle.set_transform(carla.Transform(vehicle_location, vehicle_transform.rotation))

def calculate_distance(location1, location2):
            # Calculate the Euclidean distance between two locations
            x_diff = location1.x - location2.x
            y_diff = location1.y - location2.y
            z_diff = location1.z - location2.z

            distance = math.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
            return distance 
def on_collision(event):
    # Check if the collision involves the vehicle
    #print(event.other_actor.id,vehicle.id)
    #if event.other_actor.id == vehicle.id:
    collision_sound.play()
    collision_sound.set_volume(1.0)
    my_controller.dict_file['collision']=True

user_data = {}


def save_data(userID, warning_type, timestamp, data):
    
    if userID not in user_data:
        user_data[userID] = []

    
    data_dict = {
        "timestamp": timestamp,
        "warning_type": warning_type,
        #"speed": data["speed"],
        "collision":data['collision'],
        "Reverse":data['reverse_pressed'],
        "autopilot_mode": data["autopilot_mode"],
        "lateral_stddev": data["lateral_stddev"],
        "take_over_indication":data["take_over_indication"],
        #"takeover_reaction_time": data["take_over_indication"],
        "brake_reaction_time": data["brake_reaction_time"],
        "steering_reaction_time": data["steering_reaction_time"],
        "max_longitudinal_deceleration": data["max_longitudinal_deceleration"],
        
    }

    user_data[userID].append(data_dict)
    with open('user_data.json') as f:
        data = json.load(f)

    data.update(user_data)

    
    
    with open("user_data.json", "w") as json_file:
        json.dump(data, json_file, indent=2)


host=_config['carla']['server']
port=_config['carla']['port']
front_window_size=_config['sim']['windows']['front_res']
mirror_window_size=_config['sim']['windows']['mirror_res']
autopilot=_config['carla']['autopilot']

vehicle_list=[]
mp=mirror_parameters()

# Connect to the client 
client = carla.Client(host, port)
client.set_timeout(_config['carla']['timeout'])

world = client.load_world(_config['carla']['world'])

'''
# Large World Loading
world = client.load_world('Town11')
settings = world.get_settings()
settings.tile_stream_distance = 2000
world.apply_settings(settings)
'''

# Load layered map for Town 01 with minimum layout plus buildings and parked vehicles
#world = client.load_world('Town10_Opt', carla.MapLayer.Buildings | carla.MapLayer.ParkedVehicles)
# Toggle all buildings off
#world.unload_map_layer(carla.MapLayer.Buildings)


# Getting the world and
world = client.get_world()
original_settings = world.get_settings()

# weather
weather = world.get_weather()
weather.sun_azimuth_angle = 344
weather.sun_altitude_angle = 45
weather.precipitation = 0
weather.precipitation_deposits = 0 # puddles
world.set_weather(weather)

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.no_rendering_mode=_config['carla']['no_rendering_mode']
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.02
world.apply_settings(settings)

# Set up the TM in synchronous mode
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)
tm_port = traffic_manager.get_port()
# Set a seed so behaviour can be repeated if necessary
traffic_manager.set_random_device_seed(0)
random.seed(0)

'''
# Print list of available vehicles
vehicle_blueprints = world.get_blueprint_library().filter('vehicle')
for car_bp in vehicle_blueprints:
    print (car_bp)
'''

vehicle_tag=_config['carla']['vehicle_tag']

# Instanciating te vehicle to which we attached the sensors
bp = world.get_blueprint_library().filter(vehicle_tag)[0]
bp.set_attribute('role_name', 'hero' )
#vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
spawn_points=world.get_map().get_spawn_points()
vehicle = world.spawn_actor(bp, spawn_points[1])
#vehicle = world.spawn_actor(bp,carla.Location(x=-31.545231, y=-4426.443848, z=0.001369) )

vehicle_list.append(vehicle)
print(world.get_actors())
vehicle.set_autopilot(autopilot)

#spectator = world.get_spectator()
spectator_bp = world.get_blueprint_library().filter(vehicle_tag)[0]
#spectator_location = vehicle.get_location() - carla.Location(x=-4, y=2)  # Adjust the offset as needed
#spectator_transform = carla.Transform(spectator_location, vehicle.get_transform().rotation)
#spectator_transform=carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=1)),vehicle.get_transform().rotation)
spectator_bp.set_attribute('role_name', 'spectator' )
spectator_car = world.spawn_actor(spectator_bp,random.choice(world.get_map().get_spawn_points()))
spectator_car.set_autopilot(True)
vehicle_list.append(spectator_car)
#print("After the spawn",world.get_actors())
#visit=False

#update_spectator_camera(vehicle, spectator_car)
#print("This is done")


#actors_obj = GetActorsFromWorld(world)
# Find the blueprint of the sensor.
mirror_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# Modify the attributes of the blueprint to set image resolution and field of view.
mirror_blueprint.set_attribute('image_size_x', str(mirror_window_size[0]))
mirror_blueprint.set_attribute('image_size_y', str(mirror_window_size[0]))
mirror_blueprint.set_attribute('fov', '110')


# Find the blueprint of the sensor.
car_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# Modify the attributes of the blueprint to set image resolution and field of view.
car_blueprint.set_attribute('image_size_x', str(front_window_size[0]))
car_blueprint.set_attribute('image_size_y', str(front_window_size[1]))
car_blueprint.set_attribute('fov', '110')



# Set the time in seconds between sensor captures
#blueprint.set_attribute('sensor_tick', '1')

# Provide the position of the sensor relative to the vehicle.
#left_mirror_transform = carla.Transform(carla.Location(x=.7, y=-1, z=1.2), carla.Rotation(yaw=-150))
#right_mirror_transform = carla.Transform(carla.Location(x=.7, y=1, z=1.2), carla.Rotation(yaw=150))


# lookup pre-defined mirror locations based on vehicle tag

lx, ly,lz = _config['sim']['mirror_location'][vehicle_tag]['left']
rx, ry, rz = _config['sim']['mirror_location'][vehicle_tag]['right']
left_mirror_transform = carla.Transform(carla.Location(x=lx, y=ly, z=lz), carla.Rotation(pitch=mp.left_pitch, yaw=mp.left_yaw))
right_mirror_transform = carla.Transform(carla.Location(x=rx, y=ry, z=rz), carla.Rotation(pitch=mp.right_pitch,yaw=mp.right_yaw))
front_view_transform = carla.Transform(carla.Location(x=0.8, z=1.7))

# Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
lmv_sensor = world.spawn_actor(mirror_blueprint, left_mirror_transform, attach_to=vehicle_list[0])
rmv_sensor = world.spawn_actor(mirror_blueprint, right_mirror_transform, attach_to=vehicle_list[0])
fv_sensor = world.spawn_actor(car_blueprint, front_view_transform, attach_to=vehicle_list[0])

# Subscribe to the sensor stream by providing a callback function, this function is
# called each time a new image is generated by the sensor.
fv_sensor.listen(lambda data: pygame_callback(data, renderObject))
rmv_sensor.listen(lambda data: process_image_data(data, "right_mirror_view", True))
lmv_sensor.listen(lambda data: process_image_data(data, "left_mirror_view", True))

collision_sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
collision_sensor = world.spawn_actor(collision_sensor_bp, carla.Transform(), attach_to=vehicle)
collision_sensor.listen(lambda event: on_collision(event))

# Game loop
crashed = False

# Get camera dimensions
image_w = car_blueprint.get_attribute("image_size_x").as_int()
image_h = car_blueprint.get_attribute("image_size_y").as_int()
renderObject = RenderObject(image_w, image_h)




pygame.init()

warning_sound = pygame.mixer.Sound("beep-warning.mp3")
engine_sound = pygame.mixer.Sound("car-driving.mp3")
throttle_sound = pygame.mixer.Sound("car-driving.mp3")
brake_sound = pygame.mixer.Sound("squeal-and-crash.mp3")
reverse_sound = pygame.mixer.Sound("reverse.mp3")
collision_sound = pygame.mixer.Sound("fast-collision.mp3")
pygame.font.init()
display = pygame.display.set_mode(front_window_size,  pygame.HWSURFACE | pygame.DOUBLEBUF, display=0 , vsync=1)  # pygame.FULLSCREEN |
# Draw black to the display
display.fill((0,0,0))
display.blit(renderObject.surface, (0,0))
pygame.display.flip()


my_controller=controller(world,vehicle,display)
font = pygame.font.Font(pygame.font.get_default_font(), 20)
font_name = 'courier' if os.name == 'nt' else 'mono'
fonts = [x for x in pygame.font.get_fonts() if font_name in x]
default_font = 'ubuntumono'
mono = default_font if default_font in fonts else fonts[0]
mono = pygame.font.match_font(mono)

#notifications = FadingText(font, (512, 40), (389, height - 40))
_info_text=[]
clock = pygame.time.Clock()
visit=True

speed= 26.7165
time_temp=6


#s=d/t
obs_loc=[carla.Location(x=2282.121094, y=3269.164062, z=0.003771)  ,carla.Location(x=-3035.935547, y=5243.277832, z=0.006028),carla.Location(x=-5472.874023, y=3186.843994, z=0.003952),carla.Location(x= -5055.659668,y=-2681.527832,z= 0.011131),
carla.Location(x=-31.273668, y=-4428.271484, z=0.008932),carla.Location(x=3144.657959,y=2101.735596,z=0.005167)]
count=0
while not crashed:
    # Advance the simulation time
    world.tick()
   
    
    my_controller.parse_vehicle_wheel()
    my_controller._control.reverse = my_controller._control.gear < 0
    # Update the display
    display.blit(renderObject.surface, (0,0))
    pygame.display.flip()


    v = vehicle.get_velocity()

    #carla_actors = actors_obj.get_dict_of_actors()
    #time.sleep(1)    
    for event in pygame.event.get():
        # If the window is closed, break the while loop
        if event.type == pygame.QUIT:
            crashed = True
        if event.type == pygame.JOYBUTTONDOWN:
                if event.button == my_controller._reverse_idx:
                    my_controller._control.gear = 1 if my_controller._control.reverse else -1
                    print ("Reverse", my_controller._control.gear )
                    if my_controller.dict_file["take_over_indication"] is not None:
                        my_controller.dict_file['reverse_pressed'] = True
        vehicle.show_debug_telemetry(True)
        world.show_vehicle_telemetry = True
        # Process Mirror Adjustments
        if event.type == pygame.KEYDOWN:
            if event.key == K_a:
                    mp.left_yaw += 1
            elif event.key == K_d:
                    mp.left_yaw += -1
            elif event.key == K_w:
                    mp.left_pitch += 1
            elif event.key == K_x:
                    mp.left_pitch += -1
            elif event.key == K_f:
                    mp.right_yaw += 1
            elif event.key == K_h:
                    mp.right_yaw += -1
            elif event.key == K_t:
                    mp.right_pitch += 1
            elif event.key == K_b:
                    mp.right_pitch += -1
            elif event.key == K_p :
                my_controller.toggle_autopilot(traffic_manager)
                    
            #world.notification('Autopilot %s' % ('On' if autopilot else 'Off'))
            left_mirror_transform = carla.Transform(carla.Location(x=lx, y=ly, z=lz), carla.Rotation(pitch=mp.left_pitch, yaw=mp.left_yaw))
            right_mirror_transform = carla.Transform(carla.Location(x=rx, y=ry, z=rz), carla.Rotation(pitch=mp.right_pitch,yaw=mp.right_yaw))
            lmv_sensor.set_transform(left_mirror_transform)
            rmv_sensor.set_transform(right_mirror_transform)

 
        #vehicle.apply_control(carla.VehicleControl())
        # TODO - Get vehicle telemetry and post to shared memory
        '''
        #v = vehicle.get_velocity()
        #Speed = (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        #print (Speed)
        '''
    location=vehicle.get_location()
    #print(vehicle.get_location())
    _info_text = [
    'Speed:   % 15.0f m/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)*0.6213711922),
    ('Autopilot: ', my_controller.autopilot)
    ]
    
    #lateral_stddev = my_controller.calculate_lateral_position_stddev()
    
    #if lateral_stddev is not None:
    #    _info_text.append('Lateral Position StdDev: {:.2f} meters'.format(lateral_stddev))
    

    y_position = 10
    for text in _info_text:
        
        my_controller.render_text(text, (30, y_position), font)
        y_position += 18

    print("This is my dict file", my_controller.dict_file)
    if len(obs_loc)!=0 or count==6:
        if count!=6:
            if calculate_distance(location,obs_loc[0])<=speed*time_temp and (my_controller.autopilot):
             
                 my_controller.render_text("location", (30, y_position), font)
                 flag_temp=True
                 #print("This is after",flag_temp)
                 obs_loc.pop(0)
                 count+=1

        
       # if len(my_controller.dict_file)==8 and flag_temp: 

        if my_controller.dict_file['take_over_indication'] and (time.time()-my_controller.dict_file['take_over_indication'])>=20:
            
                #print("this is the time",my_controller.dict_file['take_over_indication'])
                #print(my_controller.dict_file)

                save_data(input_user,count,time.time(),my_controller.dict_file)

                my_controller.dict_file.update({}.fromkeys(my_controller.dict_file,None))
                my_controller.longitudinal_deceleration=[]
                my_controller.lateral_positions=[]
                my_controller.dict_time_speed.clear()
                #my_controller.dict_file.clear()
                flag_temp=False

                  
                    
       # elif len(my_controller.dict_file)>=7 : 
                    #save_data(1,count,time.time(),my_controller.dict_file)
                    #my_controller.dict_file={}
             #print("This is location",obs_loc)

    #if (int(location.x) in range(2652,2750) and int(location.y) in range(2773,2797)) and (my_controller.autopilot):
     #         print("this is the location",location)
      #        my_controller.render_text("location", (30, y_position), font)    
    
    #if location.x in range(2501,2511) and location.y in range(2352,2362) and location.z in range(0,1,0.002813) and visit:
            
    
print('Shutting Down')
#print(json.dumps(user_data, indent=2))
client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
world.apply_settings(original_settings)
pygame.mixer.quit()
pygame.quit()
print ("Done")