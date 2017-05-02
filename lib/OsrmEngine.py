import os
import requests
import json
import time
from subprocess import Popen, PIPE

class OsrmEngine(object):
    """ Class which connects to an osrm-routed executable """
    def __init__(self,
                 exe_loc,
                 map_loc,
                 ghost = '0.0.0.0',
                 gport = 5000):
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
            return "The routing server \"http://%s:%d\" starts running" % (self.ghost, self.gport)
        else:
            raise Exception("Map could not be loaded")