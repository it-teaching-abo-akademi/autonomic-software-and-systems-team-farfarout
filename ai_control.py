#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math
import numpy as np

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import ai_knowledge as data
from ai_knowledge import Status
import random

# Executor is responsible for moving the vehicle around
# In this implementation it only needs to match the steering and speed so that we arrive at provided waypoints
# BONUS TODO: implement different speed limits so that planner would also provide speed target speed in addition to direction
class Executor(object):
  def __init__(self, knowledge, vehicle):
    self.vehicle = vehicle
    self.knowledge = knowledge
    self.target_pos = knowledge.get_location()
    
  #Update the executor at some intervals to steer the car in desired direction
  def update(self, time_elapsed):
    status = self.knowledge.get_status()
    #TODO: this needs to be able to handle
    if status == Status.DRIVING:
      dest = self.knowledge.get_current_destination()
      self.update_control(dest, [1], time_elapsed)

  # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
  # TODO: Take into account that exiting the crash site could also be done in reverse, so there might need to be additional data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us which things we can do (for example going in reverse)
  def update_control(self, destination, additional_vars, delta_time):
      #calculate throttle and heading
      control = carla.VehicleControl()

      max_speed = self.knowledge.retrieve_data('max_speed')
      speed = self.knowledge.retrieve_data('speed')

      if max_speed > 0 and self.knowledge.distance(destination, self.knowledge.get_location()) > 1:

        position = self.knowledge.get_location()

        heading = self.knowledge.retrieve_data('rotation').get_forward_vector()
        heading_vec = np.array([heading.x, heading.y, heading.z])
        
        # Steering
        destination_vec = destination - position
        destination_vec = np.array([destination_vec.x, destination_vec.y, destination_vec.z])

        destination_norm = destination_vec / np.linalg.norm(destination_vec)

        cross = np.cross(heading_vec, destination_norm) 
        dot_prod = min(max(np.dot(heading_vec, destination_norm), -1), 1)
        angle = math.degrees(math.acos(dot_prod))

        if (np.dot(np.array([0,0,1]), cross)) < 0:
          angle = -angle

        max_steering = self.knowledge.retrieve_data('max_steering')

        # Use some damping
        steering_angle = angle / (max_steering*1.2)
        control.steer = steering_angle

        # Throttle and brake
        error = (max_speed - speed) / max_speed
        
        if (error > 0):
          control.throttle = 1-error*0.5
          control.brake = 0.0
        else:
          control.throttle = 0.0
          control.brake = error*0.5

      else:
        control.throttle = 0.0
        control.brake = 1.0

      control.hand_brake = False
      self.vehicle.apply_control(control)


# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
class Planner(object):
  def __init__(self, knowledge):
    self.knowledge = knowledge
    self.path = deque([])

  # Create a map of waypoints to follow to the destination and save it
  def make_plan(self, source, destination):
    self.path = self.build_path(source,destination)
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())
  
  # Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())
  
  #Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
  def update_plan(self):
    if len(self.path) == 0:
      return
    
    if self.knowledge.arrived_at(self.path[0]):
      self.path.popleft()
    
    if len(self.path) == 0:
      self.knowledge.update_status(Status.ARRIVED)
    else:
      self.knowledge.update_status(Status.DRIVING)

  #get current destination 
  def get_current_destination(self):
    status = self.knowledge.get_status()
    #if we are driving, then the current destination is next waypoint
    if status == Status.DRIVING:
      #TODO: Take into account traffic lights and other cars
      if self.knowledge.retrieve_data('lidar_close') != 0:
        status = Status.HEALING
        return self.knowledge.get_location()

      return self.path[0]
    if status == Status.ARRIVED:
      return self.knowledge.get_location()
    if status == Status.HEALING:
      #TODO: Implement crash handling. Probably needs to be done by following waypoint list to exit the crash site.
      #Afterwards needs to remake the path.
      if self.knowledge.retrieve_data('lidar_close') == 0:
        status = Status.DRIVING
        return self.knowledge.get_location()
    if status == Status.CRASHED:
      #TODO: implement function for crash handling, should provide map of wayoints to move towards to for exiting crash state. 
      #You should use separate waypoint list for that, to not mess with the original path. 
      return self.knowledge.get_location()
    #otherwise destination is same as current position
    return self.knowledge.get_location()

  #TODO: Implementation
  def build_path(self, source, destination):
    self.path = deque([])
    #TODO: create path of waypoints from source to

    _map = self.knowledge.retrieve_data('map')
    waypoint = _map.get_waypoint(source.location)

    while True:
      if self.knowledge.distance(waypoint.transform.location, destination) < 30:
        nexts = list(waypoint.next(5.0))
        if waypoint.get_right_lane() and waypoint.get_right_lane().lane_type == "driving":
          nexts.extend(list(waypoint.get_right_lane().next(5.0)))
        if waypoint.get_left_lane() and waypoint.get_left_lane().lane_type == "driving":
          nexts.extend(list(waypoint.get_left_lane().next(5.0)))
      else:
        nexts = list(waypoint.next(10.0))
        if nexts[0].is_intersection: 
          # Add more options if at junction       
          nexts.extend(list(waypoint.next(5.0)))
          nexts.extend(list(waypoint.next(15.0)))

      closest = 0
      closestDistance = self.knowledge.distance(destination, nexts[0].transform.location)

      # If multiple choices
      if len(nexts) > 1:       
        for i in range(0, len(nexts)):
          dist = self.knowledge.distance(destination, nexts[i].transform.location)
          if dist < closestDistance:
            closest = i
            closestDistance = dist
        
      waypoint = nexts[closest]
      waypoint_loc = waypoint.transform.location
      self.path.append(waypoint_loc)

      # Drawing the path
      self.knowledge.retrieve_data('world').debug.draw_point(waypoint_loc, color=carla.Color(r=0, g=0, b=255), life_time=20.0)
      
      if self.knowledge.distance(waypoint_loc, destination) < 10:
        print "Path found"
        break

    self.path.append(destination)

    return self.path