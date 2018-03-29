"""
Open Source Routing Machine (OSRM)
"""

import os
import requests
import json
import time
import math
import numpy as np
from subprocess import Popen, PIPE

from lib.Constants import *
from local import hostport, osrm_version

class OsrmEngine(object):
    """
    OsrmEngine is the class for the routing server
    Attributes:
        exe_loc: path of the routing server (osrm-routed executable) (irrelevant if using singularity)
        map_loc: path of the road network file (if using singularity, path of the .osm.pbf file)
        use_singularity: true if using singularity to access osrm-backend image
        simg_loc: path of the singularity image file
        ghost: host ip address
        gport: host port
        cst_speed: constant vehicle speed when road network is disabled (in meters/second
    """
    def __init__(self,
                 exe_loc,
                 map_loc,
                 use_singularity=False,
                 simg_loc=None,
                 ghost = hostport,
                 gport = 5000,
                 cst_speed = CST_SPEED):
        if not use_singularity and not os.path.isfile(exe_loc):
            raise Exception("Could not find the routing server at %s" % exe_loc)
        else:
            self.exe_loc = exe_loc

        if not os.path.isfile(map_loc):
            raise Exception("Could not find osrm road network data at %s" % map_loc)
        else:
            self.map_loc = map_loc

        if use_singularity and simg_loc is None:
            raise Exception("If using singularity, image to osrm-backend singularity image must be provided")
        elif use_singularity and not os.path.isfile(simg_loc):
            raise Exception("Could not find osrm-backend singularity image at %s" % simg_loc)
        else:
            self.use_singularity = use_singularity
            self.simg_loc = simg_loc

        self.ghost = ghost
        self.gport = gport
        self.cst_speed = cst_speed
        # remove any open instance
        if self.check_server():
            self.kill_server()

    # kill any routing server currently running before starting something new
    def kill_server(self):
        if self.use_singularity:
            self.process.terminate()
        elif os.name == 'nt':
            # os.kill(self.pid, 1) # Kill process on windows
            os.system("taskkill /f /im osrm-routed.exe") # Kill process on windows
        else:
            Popen(["killall", os.path.basename(self.exe_loc)], stdin=PIPE, stdout=PIPE, stderr=PIPE) # Kill process on Mac/Unix
        time.sleep(2)
        self.process = None
        self.pid = None
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
        if self.use_singularity: # If we are running on MGHPCC or another environment with a singularity image of osrm-backend
            map_directory = os.path.dirname(self.map_loc)
            map_basename = os.path.basename(self.map_loc).split('.')[0]

            from spython.main import Client
            Client.execute(self.simg_loc, ['osrm-extract','-p','/opt/car.lua',self.map_loc])

            osrm_map = os.path.join(map_directory, (map_basename + '.osrm'))
            if not os.path.isfile(osrm_map):
                raise Exception("%s failed to create during osrm-extract" % osrm_map)
            
            Client.execute(self.simg_loc, ['osrm-partition', osrm_map])
            Client.execute(self.simg_loc, ['osrm-customize', osrm_map])
            Client.execute(self.simg_loc, ['osrm-contract', osrm_map])

            import multiprocessing

            def run_simg_server(simg_loc, osrm_map):
                Client.execute(simg_loc, ['osrm-routed','--algorithm','mld','-p',str(self.gport),osrm_map])

            print('About to start server.')
            server_process = multiprocessing.Process(
                name='server', target=run_simg_server, args=(self.simg_loc, osrm_map))

            print('Starting server....')
            server_process.start()
            self.process = server_process

            time.sleep(2)

            if server_process.is_alive() and requests.get("http://%s:%d" % (self.ghost, self.gport)).status_code == 400:
                print( "The routing server \"http://%s:%d\" starts running" % (self.ghost, self.gport) )
            else:
                raise Exception("Map could not be loaded")

        else: # If we are running locally and do not need to use singularity to access osrm-backend
            # check file
            try:
                p = Popen([self.exe_loc, '-v'], stdin=PIPE, stdout=PIPE, stderr=PIPE)
                output = p.communicate()[0].decode("utf-8")
            except FileNotFoundError:
                output = ""
            if osrm_version not in str(output):
                raise Exception("osrm does not have the right version")
            # check no running server
            if self.check_server():
                raise Exception("osrm-routed already running")
            # start server
            p = Popen([self.exe_loc, '-p', str(self.gport), self.map_loc], stdin=PIPE, stdout=PIPE, stderr=PIPE)
            self.pid = p.pid
            time.sleep(5)
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
        # while count < 10:
        try:
            response = requests.get(url, timeout=100)
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
        print("The routing server \"http://%s:%d\" failed and has been restarted... :(" % (self.ghost, self.gport) )
        return self.call_url(url)

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
            return (6371000*2*math.pi/360 * np.sqrt( (math.cos((olat+dlat)*math.pi/360)*(olng-dlng))**2 + (olat-dlat)**2))
    
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
            
