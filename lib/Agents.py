import numpy as np
from collections import deque
import matplotlib.pyplot as plt

class Step(object):
    """ 
    Step is a class for steps in a leg
    Attributes:
        d: distance 
        c: cost in duration of time
        geo: geometry, a list of coordinates
    """
    def __init__(self, d=0.0, c=0.0, geo=[]):
        self.d = d
        self.c = c
        self.geo = geo
        
    def __str__(self):
        return "step: distance = %.1f, duration = %.1f" % (self.d, self.c)
 
    
class Leg(object):
    """ 
    Leg is a class for legs in the route
    Attributes:
        d: total distance 
        c: total cost in duration of time
        steps: a list of steps
    """
    def __init__(self, d=0.0, c=0.0, steps=[]):
        self.d = d
        self.c = c
        self.steps = deque([])
        
    def add_step(self, step):
        self.steps.append(step)
        
    def __str__(self):
        return "leg: distance = %.1f, duration = %.1f, number of steps = %d" % (self.d, self.c, len(self.steps) )
    
    
class Veh(object):
    """ 
    Veh is a class for vehicles
    Attributes:
        id: sequential unique id
        lat: current latitude
        lng: current longitude
        tlat: target (end of route) latitude
        tlng: target (end of route) longitude
        t: system time at current state
        k: capacity
        n: number of passengers on board
        jobs: a list of jobs in the format of (request id, pickup or dropoff, target lat, target lng)
        route: a list of legs
        c: total cost of the route
        d: total distance of the route
    """  
    def __init__(self, id, k=4, lat=0.089653, lng=51.373206, t=0.0):
        self.id = id
        self.lat = lat
        self.lng = lng
        self.tlat = lat
        self.tlng = lng
        self.t = t
        self.k = k
        self.n = 0
        self.jobs = deque([])
        self.route = deque([])
        self.c = 0.0
        self.d = 0.0
        
    def get_location(self):
        return (self.lat, self.lng)
    
    def move_to_location(self, lat, lng):
        self.lat = lat
        self.lng = lng
    
    def add_job(self, osrm, rid, pd, rlat, rlng):
        self.jobs.append( (rid, pd, rlat, rlng) )
        out = osrm.get_routing(self.tlat, self.tlng, rlat, rlng)
        assert len(out['legs']) == 1
        leg = Leg(out['legs'][0]['distance'], out['legs'][0]['duration'], steps=[])
        for s in range( len(out['legs'][0]['steps']) ):
            step = Step(out['legs'][0]['steps'][s]['distance'],
                        out['legs'][0]['steps'][s]['duration'],
                        out['legs'][0]['steps'][s]['geometry']['coordinates'])
            leg.add_step(step)
        assert len(step.geo) == 2
        assert step.geo[0] == step.geo[1]
        self.add_leg(leg)
        assert len(self.jobs) == len(self.route)
    
    def add_pickup_job(self, osrm, req):
        self.add_job(osrm, req.id, 1, req.olat, req.olng)
    
    def add_dropoff_job(self, osrm, req):
        self.add_job(osrm, req.id, -1, req.dlat, req.dlng)

    def add_leg(self, leg):
        self.route.append(leg)
        self.tlat = leg.steps[-1].geo[1][0]
        self.tlng = leg.steps[-1].geo[1][1]
        self.d += leg.d
        self.c += leg.c
        
    def pop_job(self):
        self.jobs.popleft()
        self.pop_leg()
        assert len(self.jobs) == len(self.route)
        
    def pop_leg(self):
        leg = self.route.popleft()
        self.d -= leg.d
        self.c -= leg.c
        
    def pop_step(self):
        step = self.route[0].steps.popleft()
        self.c -= step.c
        self.d -= step.d
        self.route[0].c -= step.c
        self.route[0].d -= step.d
        
    def cut_step(self, pct):
        step = self.route[0].steps[0]
        dis = 0
        sega = step.geo[0]
        for segb in step.geo[1:]:
            dis += np.sqrt( (sega[0] - segb[0])**2 + (sega[1] - segb[1])**2)
            sega = segb
        dis_ = 0
        sega = step.geo[0]
        for segb in step.geo[1:]:
            _dis = np.sqrt( (sega[0] - segb[0])**2 + (sega[1] - segb[1])**2)
            dis_ += _dis
            if dis_ / dis > pct:
                break
            sega = segb
        while step.geo[0] != sega:
            step.geo.pop(0)
        _pct = (pct * dis - dis_ + _dis) / _dis
        
        step.geo[0][0] = sega[0] + _pct * (segb[0] - sega[0])
        step.geo[0][1] = sega[1] + _pct * (segb[1] - sega[1])
        
        self.c -= step.c * pct
        self.d -= step.d * pct
        self.route[0].c -= step.c * pct
        self.route[0].d -= step.d * pct
        self.route[0].steps[0].c -= step.c * pct
        self.route[0].steps[0].d -= step.d * pct  
        
    def move_to_time(self, t):
        dt = t - self.t
        assert dt >= 0
        done = []
        if dt == 0 or len(self.jobs) == 0:
            self.t = t
            return done
        while dt > 0 and len(self.jobs) > 0:
            if self.route[0].c <= dt:
                dt -= self.route[0].c
                self.t += self.route[0].c
                self.move_to_location(self.jobs[0][2], self.jobs[0][3])
                done.append( (self.jobs[0][0], self.jobs[0][1], self.t) )
                self.n += self.jobs[0][1]
                self.pop_job()
            else:
                while dt > 0 and len(self.route[0].steps) > 0:
                    if self.route[0].steps[0].c <= dt:
                        dt -= self.route[0].steps[0].c
                        self.pop_step()
                    else:
                        pct = dt / self.route[0].steps[0].c
                        self.cut_step(pct)
                        self.move_to_location(self.route[0].steps[0].geo[0][0], self.route[0].steps[0].geo[0][1])
                        self.t = t
                        return done
        assert self.n == 0
        assert np.isclose(self.d, 0.0)
        assert np.isclose(self.c, 0.0)
        self.t = t
        self.d = 0.0
        self.c = 0.0
        return done
    
    def draw(self):
        plt.plot(self.lat, self.lng, 'b', marker='o')
        for l in range(len(self.route)):
            for s in range(len(self.route[l].steps)):
                step = np.transpose( self.route[l].steps[s].geo )
                plt.plot(step[0], step[1], 'b', linestyle='-' if l == 0 else '--')
                        
    def __str__(self):
        str =  "veh %d at (%.7f, %.7f) when t = %.3f; occupancy = %d/%d" % (
            self.id, self.lat, self.lng, self.t, self.n, self.k)
        str += "\n  has %d job(s), distance = %.1f, cost = %.1f" % (len(self.jobs), self.d, self.c)
        for j in self.jobs:
            str += "\n    %s: req %d at (%.7f, %.7f)" % ("pickup" if j[1] > 0 else "dropoff", j[0], j[2], j[3])
        return str

    
class Req(object):
    """ 
    Req is a class for requests
    Attributes:
        id: sequential unique id
        t: request time
        olat: origin latitude
        olng: origin longitude
        dlat: destination latitude
        dlng: destination longitude
    """
    def __init__(self, id, t, olat=0.115662, olng=51.374282, dlat=0.089282, dlng=51.350675):
        self.id = id
        self.t = t
        self.olat = olat
        self.olng = olng
        self.dlat = dlat
        self.dlng = dlng
    
    def get_origin(self):
        return (self.olat, self.olng)
    
    def get_destination(self):
        return (self.dlat, self.dlng)
    
    def __str__(self):
        return "req %d from (%.7f, %.7f) to (%.7f, %.7f) at t = %.3f" % (
            self.id, self.olat, self.olng, self.dlat, self.dlng, self.t)
        