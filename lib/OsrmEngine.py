"""
Open Source Routing Machine (OSRM)
"""

import os
import requests
import json
import time
import numpy as np
from subprocess import Popen, PIPE

from lib.Constants import *

class OsrmEngine(object):
    """
    OsrmEngine is the class for the routing server
    Attributes:
        exe_loc: path of the routing server (osrm-routed executable)
        map_loc: path of the road network file
        ghost: host ip address
        gport: host port
        cst_speed: constant vehicle speed when road network is disabled (in meters/second
    """
    def __init__(self,
                 exe_loc,
                 map_loc,
                 ghost = '0.0.0.0',
                 gport = 5000,
                 cst_speed = CST_SPEED):
        if not os.path.isfile(exe_loc):
            raise Exception("Could not find the routing server at %s" % exe_loc)
        else:
            self.exe_loc = exe_loc
        if not os.path.isfile(map_loc):
            raise Exception("Could not find osrm road network data at %s" % map_loc)
        else:
            self.map_loc = map_loc
        self.ghost = ghost
        self.gport = gport
        self.cst_speed = cst_speed
        # remove any open instance
        if self.check_server():
            self.kill_server()

    # kill any routing server currently running before starting something new
    def kill_server(self):
        Popen(["killall", os.path.basename(self.exe_loc)], stdin=PIPE, stdout=PIPE, stderr=PIPE)
        time.sleep(2)
        print( "The routing server \"http://%s:%d\" is killed" % (self.ghost, self.gport) )
        
    # check if server is already running
    def check_server(self):
        try:
             if requests.get("http://%s:%d" % (self.ghost, self.gport)).status_code == 400:
                return True
        except requests.ConnectionError:
            return False
    
    # start the routing server
    def start_server(self):
        # check file
        try:
            p = Popen([self.exe_loc, '-v'], stdin=PIPE, stdout=PIPE, stderr=PIPE)
            output = p.communicate()[0].decode("utf-8")
        except FileNotFoundError:
            output = ""
        if "v5.11.0" not in str(output):
            raise Exception("osrm does not have the right version")
        # check no running server
        if self.check_server():
            raise Exception("osrm-routed already running")
        # start server
        p = Popen([self.exe_loc, self.map_loc], stdin=PIPE, stdout=PIPE, stderr=PIPE)
        time.sleep(2)
        if requests.get("http://%s:%d" % (self.ghost, self.gport)).status_code == 400:
            print( "The routing server \"http://%s:%d\" starts running" % (self.ghost, self.gport) )
        else:
            raise Exception("Map could not be loaded")
    
    # restart the routing server        
    def restart_server(self):
        self.kill_server()
        self.start_server()
    
    # generate the request in url format        
    def create_url(self, olng, olat, dlng, dlat, steps="false", annotations="false"):
        return "http://{0}:{1}/route/v1/driving/{2},{3};{4},{5}?alternatives=false&steps={6}&annotations={7}&geometries=geojson".format(
            self.ghost, self.gport, olng, olat, dlng, dlat, steps, annotations)

    # send the request and get the response in Json format
    def call_url(self, url):
        count = 0
        while count < 10:
            try:
                response = requests.get(url, timeout=1)
                json_response = response.json()
                code = json_response['code']
                if code == 'Ok':
                    return (json_response, True)
                else:
                    print("Error: %s" % (json_response['message']))
                    return (json_response, False)
            except requests.exceptions.Timeout:
                print(url)
                self.restart_server()
                count += 1
            except Exception as err:
                print("Failed: %s" % (url))
                return (None, False)
        print("The routing server \"http://%s:%d\" fails after 10 retries... :(" % (self.ghost, self.gport) )

    # get the best route from origin to destination 
    def get_routing(self, olng, olat, dlng, dlat):
        url = self.create_url(olng, olat, dlng, dlat, steps="true", annotations="false")
        (response, code) = self.call_url(url)
        if code:
            return response['routes'][0]['legs'][0]
        else:
            return None
    
    # get the distance of the best route from origin to destination
    # if road network is not enabled, return Euclidean distance
    def get_distance(self, olng, olat, dlng, dlat):
        if IS_ROAD_ENABLED:
            url = self.create_url(olng, olat, dlng, dlat, steps="false", annotations="false")
            (response, code) = self.call_url(url)
            if code:
                return response['routes'][0]['distance']
            else:
                return None
        else:
            return np.sqrt( (69600 * (olng-dlng))**2 + (111317 * (olat-dlat))**2 )
    
    # get the duration of the best route from origin to destination
    # if road network is not enabled, return the duration based on Euclidean distance and constant speed   
    def get_duration(self, olng, olat, dlng, dlat):
        if IS_ROAD_ENABLED:
            url = self.create_url(olng, olat, dlng, dlat, steps="false", annotations="false")
            (response, code) = self.call_url(url)
            if code:
                return response['routes'][0]['duration']
            else:
                return None
        else:
            return self.get_distance(olng, olat, dlng, dlat) / self.cst_speed
    
    # get both distance and duration
    def get_distance_duration(self, olng, olat, dlng, dlat):
        if IS_ROAD_ENABLED:
            url = self.create_url(olng, olat, dlng, dlat, steps="false", annotations="false")
            (response, code) = self.call_url(url)
            if code:
                return (response['routes'][0]['distance'], response['routes'][0]['duration'])
            else:
                return None
        else:
            return self.get_distance(olng, olat, dlng, dlat), self.get_duration(olng, olat, dlng, dlat) 
            
