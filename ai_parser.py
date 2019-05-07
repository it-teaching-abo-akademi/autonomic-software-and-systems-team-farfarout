#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import weakref
import carla
import ai_knowledge as data
import numpy as np


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
# TODO: Implement other sensors (lidar and depth sensors mainly)
# TODO: Use carla API to read whether car is at traffic lights and their status, update it into knowledge
class Monitor(object):
  def __init__(self, knowledge,vehicle):
    self.vehicle = vehicle
    self.knowledge = knowledge
    weak_self = weakref.ref(self)
    
    self.knowledge.update_data('location', self.vehicle.get_transform().location)
    self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
    self.knowledge.update_data('max_steering', self.vehicle.get_physics_control().wheels[0].steer_angle)
    self.knowledge.update_data('topology', self.vehicle.get_world().get_map().get_topology())
    self.knowledge.update_data('map', self.vehicle.get_world().get_map())
    self.knowledge.update_data('lidar', 0);
    self.knowledge.update_data('bb', self.vehicle.bounding_box)

    # For debugging
    self.knowledge.update_data('world', self.vehicle.get_world())

    # Drawing all segments
    #for segment in self.vehicle.get_world().get_map().get_topology():
    #  self.vehicle.get_world().debug.draw_line(segment[0].transform.location, segment[1].transform.location,
    #                                   color=carla.Color(r=255, g=0, b=0), life_time=120.0)

    world = self.vehicle.get_world()
    bp = world.get_blueprint_library().find('sensor.other.lane_detector')
    self.lane_detector = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
    self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

    bp2 = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    bp2.set_attribute('range', '1000')
    bp2.set_attribute('points_per_second', '1000')
    bp2.set_attribute('channels', '9')
    bp2.set_attribute('upper_fov', '0')
    bp2.set_attribute('lower_fov', '-45')
    bp2.set_attribute('sensor_tick', '0.5')
    #bp2.set_attribute('rotation_frequency', '360')
    self.lidar = world.spawn_actor(bp2, carla.Transform(), attach_to=self.vehicle)
    self.lidar.set_transform(carla.Transform(carla.Location(z=2)))
    self.lidar.listen(lambda event: Monitor._lidar_update(weak_self, event))


  #Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    # Update the position of vehicle into knowledge
    self.knowledge.update_data('location', self.vehicle.get_transform().location)
    self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
    self.knowledge.update_data('velocity', self.vehicle.get_velocity())
    self.knowledge.update_data('at_lights', self.vehicle.is_at_traffic_light())
    self.knowledge.update_data('traffic_light_state', self.vehicle.get_traffic_light_state())

  @staticmethod
  def _on_invasion(weak_self, event):
    self = weak_self()
    if not self:
      return
    self.knowledge.update_data('lane_invasion',event.crossed_lane_markings)

  @staticmethod
  def _lidar_update(weak_self, event):
    self = weak_self()
    if not self:
      return
    self.knowledge.update_data('lidar', event)

# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
# TODO: During the update step parse the data inside knowledge into information that could be used by planner to plan the route
class Analyser(object):
  def __init__(self, knowledge):
    self.knowledge = knowledge

  #Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    velocity = self.knowledge.retrieve_data('velocity')
    speed = np.linalg.norm(np.array([velocity.x, velocity.y, velocity.z]))
    self.knowledge.update_data('speed', speed)

    lidar = self.knowledge.retrieve_data('lidar')
    car_loc = self.knowledge.retrieve_data('location')
    bb = self.knowledge.retrieve_data('bb')

    if not lidar == 0:     
      for location in lidar:
          #print location
          rel_loc = carla.Location(car_loc.x+location.x, car_loc.y+location.y, car_loc.z+location.z+2)
          self.knowledge.retrieve_data('world').debug.draw_point(rel_loc,
                                        color=carla.Color(r=255, g=0, b=0), life_time=1.0)

    self.knowledge.update_data('max_speed', 5)

    if self.knowledge.retrieve_data('at_lights'):
      if self.knowledge.retrieve_data('traffic_light_state') == carla.TrafficLightState.Red:
        self.knowledge.update_data('max_speed', 0)    
    
    return
