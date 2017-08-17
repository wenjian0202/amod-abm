import os
import requests
import json
import time
from subprocess import Popen, PIPE

from lib.Constants import *

class OsrmEngine(object):
    """ Class which connects to an osrm-routed executable """
    def __init__(self,
                 exe_loc,
                 map_loc,
                 ghost = '0.0.0.0',
                 gport = 5000,
                 cst_speed = 6):
        """ Map needs to be pre-processed using osrm-extract and osrm-contract """
        if not os.path.isfile(exe_loc):
            raise Exception("Could not find osrm-routed executable at %s" % exe_loc)
        else:
            self.exe_loc = exe_loc
        if not os.path.isfile(map_loc):
            raise Exception("Could not find osrm data at %s" % map_loc)
        else:
            self.map_loc = map_loc
        self.ghost = ghost
        self.gport = gport
        self.cst_speed = cst_speed
        # Remove any open instance
        if self.check_server():
            self.kill_server()
 
    def kill_server(self):
        """
        Kill any osrm-routed server that is currently running before spawning new - this means only one script
        can be run at a time
        """
        Popen(["killall", os.path.basename(self.exe_loc)], stdin=PIPE, stdout=PIPE, stderr=PIPE)
        time.sleep(2)
        print( "The routing server \"http://%s:%d\" is killed" % (self.ghost, self.gport) )
        
    def check_server(self):
        """ Check if server is already running """
        try:
             if requests.get("http://%s:%d" % (self.ghost, self.gport)).status_code == 400:
                return True
        except requests.ConnectionError:
            return False
        
    def start_server(self):
        """ Initialize the routing server """
        # Check osrm-routed executable
        try:
            p = Popen([self.exe_loc, '-v'], stdin=PIPE, stdout=PIPE, stderr=PIPE)
            output = p.communicate()[0].decode("utf-8")
        except FileNotFoundError:
            output = ""
        if "v5.5.0" not in str(output):
            raise Exception("osrm does not have the right version")
        # Check server not running
        if self.check_server():
            raise Exception("osrm-routed already running")
        # Start server
        p = Popen([self.exe_loc, self.map_loc], stdin=PIPE, stdout=PIPE, stderr=PIPE)
        time.sleep(2)
        if requests.get("http://%s:%d" % (self.ghost, self.gport)).status_code == 400:
            print( "The routing server \"http://%s:%d\" starts running" % (self.ghost, self.gport) )
        else:
            raise Exception("Map could not be loaded")
            
    def restart_server(self):
        self.kill_server()
        self.start_server()
            
    def create_url(self, olng, olat, dlng, dlat, steps="false", annotations="false"):
        return "http://{0}:{1}/route/v1/driving/{2},{3};{4},{5}?alternatives=false&steps={6}&annotations={7}&geometries=geojson".format(
            self.ghost, self.gport, olng, olat, dlng, dlat, steps, annotations)

    def call_url(self, url):
        """ Send the request and get the response in Json format """
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

    def get_routing(self, olng, olat, dlng, dlat):
        url = self.create_url(olng, olat, dlng, dlat, steps="true", annotations="false")
        (response, code) = self.call_url(url)
        if code:
            return response['routes'][0]['legs'][0]
        else:
            return None
    
    def get_distance(self, olng, olat, dlng, dlat):
        if ROAD_ENABLED:
            url = self.create_url(olng, olat, dlng, dlat, steps="false", annotations="false")
            (response, code) = self.call_url(url)
            if code:
                return response['routes'][0]['distance']
            else:
                return None
        else:
            return np.sqrt( (69600 * (lng1-lng2))**2 + (111317 * (lat1-lat2))**2 )
        
    def get_duration(self, olng, olat, dlng, dlat):
        if ROAD_ENABLED:
            url = self.create_url(olng, olat, dlng, dlat, steps="false", annotations="false")
            (response, code) = self.call_url(url)
            if code:
                return response['routes'][0]['duration']
            else:
                return None
        else:
            return self.get_distance(olng, olat, dlng, dlat) / self.cst_speed
    
    def get_distance_duration(self, olng, olat, dlng, dlat):
        if ROAD_ENABLED:
            url = self.create_url(olng, olat, dlng, dlat, steps="false", annotations="false")
            (response, code) = self.call_url(url)
            if code:
                return (response['routes'][0]['distance'], response['routes'][0]['duration'])
            else:
                return None
        else:
            return self.get_distance(olng, olat, dlng, dlat), self.get_duration(olng, olat, dlng, dlat) 