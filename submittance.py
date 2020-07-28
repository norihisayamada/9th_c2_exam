#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_import_export.py: 

This example demonstrates how to import and export files in the Waypoint file format 
(http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format). The commands are imported
into a list, and can be modified before saving and/or uploading.

Documentation is provided at http://python.dronekit.io/examples/mission_import_export.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


#Set up option parsing to get connection string
# import argparse  
# parser = argparse.ArgumentParser(description='Demonstrates mission import/export from a file.')
# parser.add_argument('--connect', 
#                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
# args = parser.parse_args()

# connection_string = args.connect
# sitl = None

#Start SITL if no connection string specified
# if not connection_string:
#     import dronekit_sitl
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()


# Connect to the Vehicle
# print('Connecting to vehicle on: %s' % connection_string)
# vehicle = connect(connection_string, wait_ready=True)


vehicle = connect('127.0.0.1:14550', wait_ready=True)
print("connected")

cmds = vehicle.commands
print("Clear any existing commands")
cmds.clear() 


#  Check that vehicle is armable. 
# This ensures home_location is set (needed when saving WP file)
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)


def readmission(aFileName):
    
    # Load a mission from a file into a list. The mission definition is in the Waypoint file
    # format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    # This function is used by upload_mission().
    
    print("\nReading mission from file: %s" % aFileName)
    #cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                #ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist



def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Clear existing mission from vehicle
    print('Clear mission')
    cmds = vehicle.commands
    cmds.clear()

    #Read mission from file
    missionlist = readmission(aFileName)
    print("Upload mission from a file: %s" % aFileName)
   
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print("Uploaded commands to vehicle\n")
    cmds.upload()
    cmds.wait_ready()
    
# import_mission_filename = 'mission_9th.waypoints'
#import_mission_filename = 'nose_dronefield.waypoints'
# import_mission_filename = 'nose_rover.waypoints'
import_mission_filename = 'A-2-2_changeport.waypoints'

export_mission_filename = 'exportedmission.txt'
#Upload mission from file
upload_mission(import_mission_filename)


while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    if not vehicle.home_location:
        print('Waiting for home location')
    print('\nHomeLocation: %s' % vehicle.home_location)


def arm_and_takeoff(aTargetAltitude):
    
    #Arms vehicle and fly to aTargetAltitude.
   

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        #alt = aTargetAltitude
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
  

# From Copter 3.3 you will be able to take off using a mission item.
arm_and_takeoff(20)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

while True:
    nextwaypoint=vehicle.commands.next
    print('waypoint (%s)' % (nextwaypoint, ))
    
    if nextwaypoint==27:
        print("Exit mission when start heading to final waypoint (%s)" % (nextwaypoint, ))
        break
              
    time.sleep(1)

print('Return to launch')
vehicle.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
