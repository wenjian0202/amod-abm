import numpy as np
import copy
from collections import deque
import matplotlib.pyplot as plt

from lib.Demand import *
from lib.Constants import *

class Step(object):
    """ 
    Step is a class for steps in a leg
    Attributes:
        d: distance 
        t: duration
        geo: geometry, a list of coordinates
    """
    def __init__(self, d=0.0, t=0.0, geo=[]):
        self.d = d
        self.t = t
        self.geo = geo
        
    def __str__(self):
        return "step: distance = %.1f, duration = %.1f" % (self.d, self.t)
 
    
class Leg(object):
    """ 
    Leg is a class for legs in the route
    Attributes:
        rid: request id (if rebalancing then -1)
        pod: pickup (+1) or dropoff (-1), rebalancing (0)
        tlng: target longitude
        tlat: target latitude
        d: total distance 
        t: total duration
        steps: a list of steps
    """
    def __init__(self, rid, pod, tlng, tlat, d=0.0, t=0.0, steps=[]):
        self.rid = rid
        self.pod = pod
        self.tlng = tlng
        self.tlat = tlat
        self.d = d
        self.t = t
        self.steps = deque([])
        
    def __str__(self):
        return "leg: distance = %.1f, duration = %.1f, number of steps = %d" % (self.d, self.t, len(self.steps) )
    

class Veh(object):
    """ 
    Veh is a class for vehicles
    Attributes:
        id: sequential unique id
        T: system time at current state
        lat: current lngitude
        lng: current longtitude
        tlat: target (end of route) lngitude
        tlng: target (end of route) longtitude
        K: capacity
        n: number of passengers on board
        route: a list of legs
        t: total duration of the route
        d: total distance of the route
        c: total cost (generalized time) of the passegners
        Ds: accumulated service distance traveled
        Ts: accumulated service time traveled
        Dr: accumulated rebalancing distance traveled
        Tr: accumulated rebalancing time traveled
    """ 
    def __init__(self, id, K=4, lng=0.080444, lat=51.381263, T=0.0):
        self.id = id
        self.T = T
        self.lat = lat + np.random.normal(0.0, 0.015)
        self.lng = lng + np.random.normal(0.0, 0.020) 
        self.tlat = lat
        self.tlng = lng
        self.K = K
        self.n = 0
        self.route = deque([])
        self.t = 0.0
        self.d = 0.0
        self.c = 0.0
        self.Ds = 0.0
        self.Ts = 0.0
        self.Dr = 0.0
        self.Tr = 0.0
        
    def get_location(self):
        return (self.lng, self.lat)
    
    def get_target_location(self):
        return (self.tlng, self.tlat)
    
    def jump_to_location(self, lng, lat):
        self.lng = lng
        self.lat = lat
        
    def build_route(self, osrm, route):
        self.clear_route()
        for (rid, pod, tlng, tlat) in route:
            self.add_leg(osrm, rid, pod, tlng, tlat)
        self.update_cost()
        
    def clear_route(self):
        self.route.clear()
        self.d = 0.0
        self.t = 0.0
        self.c = 0.0
        self.tlng = self.lng
        self.tlat = self.lat
    
    def add_leg(self, osrm, rid, pod, tlng, tlat):
        out = osrm.get_routing(self.tlng, self.tlat, tlng, tlat)
        assert len(out['legs']) == 1
        leg = Leg(rid, pod, tlng, tlat, 
                  out['legs'][0]['distance'], out['legs'][0]['duration'], steps=[])
        for s in out['legs'][0]['steps']:
            step = Step(s['distance'], s['duration'], s['geometry']['coordinates'])
            leg.steps.append(step)
        assert len(step.geo) == 2
        assert step.geo[0] == step.geo[1]
        self.route.append(leg)
        self.tlng = leg.steps[-1].geo[1][0]
        self.tlat = leg.steps[-1].geo[1][1]
        self.d += leg.d
        self.t += leg.t
        
    def update_cost(self):
        c = 0.0
        t = 0.0
        n = self.n
        for leg in self.route:
            t += leg.t
            c += n * leg.t * B_V
            n += leg.pod
            c += t * B_W if leg.pod == 1 else 0
        self.c = c
        
    def move_to_time(self, T):
        dT = T - self.T
        if dT <= 0:
            return []
        done = []
        while dT > 0 and len(self.route) > 0:
            leg = self.route[0]
            if leg.t < dT:
                dT -= leg.t
                self.T += leg.t
                if self.T >= WARM_UP and self.T <= WARM_UP+SIMULATION:
                    self.Ts += leg.t if leg.rid != -1 else 0
                    self.Ds += leg.d if leg.rid != -1 else 0
                    self.Tr += leg.t if leg.rid == -1 else 0
                    self.Dr += leg.d if leg.rid == -1 else 0
                self.jump_to_location(leg.tlng, leg.tlat)
                self.n += leg.pod
                done.append( (leg.rid, leg.pod, self.T) )
                self.pop_leg()
            else:
                while dT > 0 and len(self.route[0].steps) > 0:
                    step = self.route[0].steps[0]
                    if step.t < dT:
                        dT -= step.t
                        if self.T >= WARM_UP and self.T <= WARM_UP+SIMULATION:
                            self.Ts += step.t if leg.rid != -1 else 0
                            self.Ds += step.d if leg.rid != -1 else 0
                            self.Tr += step.t if leg.rid == -1 else 0
                            self.Dr += step.d if leg.rid == -1 else 0
                        self.pop_step()
                    else:
                        pct = dT / step.t
                        if self.T >= WARM_UP and self.T <= WARM_UP+SIMULATION:
                            self.Ts += dT if leg.rid != -1 else 0
                            self.Ds += step.d * pct if leg.rid != -1 else 0
                            self.Tr += dT if leg.rid == -1 else 0
                            self.Dr += step.d * pct if leg.rid == -1 else 0
                        self.cut_step(pct)
                        self.jump_to_location(step.geo[0][0], step.geo[0][1])
                        self.T = T
                        return done
        assert dT > 0
        assert len(self.route) == 0 and self.n == 0
        assert np.isclose(self.d, 0.0)
        assert np.isclose(self.t, 0.0)
        self.T = T
        self.d = 0.0
        self.t = 0.0
        return done
        
    def pop_leg(self):
        leg = self.route.popleft()
        self.d -= leg.d
        self.t -= leg.t
        
    def pop_step(self):
        step = self.route[0].steps.popleft()
        self.t -= step.t
        self.d -= step.d
        self.route[0].t -= step.t
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
        
        self.t -= step.t * pct
        self.d -= step.d * pct
        self.route[0].t -= step.t * pct
        self.route[0].d -= step.d * pct
        self.route[0].steps[0].t -= step.t * pct
        self.route[0].steps[0].d -= step.d * pct  
    
    def draw(self):
        plt.plot(self.lng, self.lat, 'b', marker='o')
        count = 0
        for leg in self.route:
            count += 1
            plt.plot(leg.tlng, leg.tlat, 'b', 
                     marker='+' if leg.pod == 1 else 'x' if leg.pod == -1 else '.')
            for step in leg.steps:
                geo = np.transpose( step.geo )
                plt.plot(geo[0], geo[1], 'b', linestyle='-' if count<=1 else '--')

                        
    def __str__(self):
        str =  "veh %d at (%.7f, %.7f) when t = %.3f; occupancy = %d/%d;" % (
            self.id, self.lng, self.lat, self.T, self.n, self.K)
        str += "\n  service distance/time travelled: %.1f, %.1f; rebalancing distance/time travelled: %.1f, %.1f" % (
            self.Ds, self.Ts, self.Dr, self.Tr)
        str += "\n  has %d leg(s), distance = %.1f, duration = %.1f，cost = %.1f" % (
            len(self.route), self.d, self.t, self.c)
        for leg in self.route:
            str += "\n    %s: req %d at (%.7f, %.7f), distance = %.1f, duration = %.1f" % (
                "pickup" if leg.pod == 1 else "dropoff" if leg.pod == -1 else "rebalancing",
                leg.rid, leg.tlng, leg.tlat, leg.d, leg.t)
        return str
    

class Req(object):
    """ 
    Req is a class for requests
    Attributes:
        id: sequential unique id
        Tr: request time
        olat: origin lngitude
        olng: origin longtitude
        dlng: destination longtitude
        dlat: destination lngitude
        Clp: constraint - latest pickup
        Cld: constraint - latest dropoff
        Tp: pickup time
        Td: dropoff time
    """
    def __init__(self, osrm, id, Tr, olat=51.374282, olng=0.115662, dlat=51.350675, dlng=0.089282):
        self.id = id
        self.Tr = Tr
        self.olat = olat
        self.olng = olng
        self.dlat = dlat
        self.dlng = dlng
        self.Clp = Tr + MAX_WAIT
        self.Cld = self.Clp + MAX_DETOUR * osrm.get_duration(olng, olat, dlng, dlat)
        self.Tp = -1.0
        self.Td = -1.0
    
    def get_origin(self):
        return (self.olng, self.olat)
    
    def get_destination(self):
        return (self.dlng, self.dlat)
    
    def draw(self):
        plt.plot(self.olng, self.olat, 'r', marker='+')
        plt.plot(self.dlng, self.dlat, 'r', marker='x')
        plt.plot([self.olng, self.dlng], [self.olat, self.dlat], 'r', linestyle='--', dashes=(0.5,1.5))
    
    def __str__(self):
        str = "req %d from (%.7f, %.7f) to (%.7f, %.7f) at t = %.3f" % (
            self.id, self.olng, self.olat, self.dlng, self.dlat, self.Tr)
        str += "\n  latest pickup at t = %.3f, latest dropoff at t = %.3f" % ( self.Clp, self.Cld)
        str += "\n  pickup at t = %.3f, dropoff at t = %.3f" % ( self.Tp, self.Td)
        return str
    

class Model(object):
    """
    Model is the class for the AMoD system
    Attributes:
        T: system time at current state
        D: average arrival interval (sec)
        demand: demand matrix
        V: number of vehicles
        K: capacity of vehicles
        vehs: the list of vehicles
        N: number of requests
        reqs: the list of requests
        queue: requests in the queue
    """ 
    def __init__(self, D, demand, V=2, K=4):
        self.T = 0.0
        self.D = D
        self.demand = demand
        self.V = V
        self.K = K
        self.vehs = []
        for i in range(V):
            self.vehs.append(Veh(i, K=K))
        self.N = 0
        self.reqs = []
        self.queue = deque([])
        
    def generate_request(self, osrm):
        dt = self.D * np.random.exponential()
        rand = np.random.rand()
        for d in demand:
            if d[4] > rand:
                req = Req(osrm, 
                          0 if self.N == 0 else self.reqs[-1].id+1,
                          dt if self.N == 0 else self.reqs[-1].Tr+dt,
                          d[0], d[1], d[2], d[3])
                break
        self.N += 1
        return req
        
    def generate_requests_to_time(self, osrm, T):
        if self.N == 0:
            req = self.generate_request(osrm)
            self.reqs.append(req)
        while self.reqs[-1].Tr <= T:
            req = self.generate_request(osrm)
            self.queue.append(self.reqs[-1])
            self.reqs.append(req)
        assert self.N == len(self.reqs)
        
    def dispatch_at_time(self, osrm, T):
        self.T = T
        for v in self.vehs:
            done = v.move_to_time(T)
            for (rid, pod, t) in done:
                if pod == 1:
                    self.reqs[rid].Tp = t
                elif pod == -1:
                    self.reqs[rid].Td = t
            v.update_cost()
        self.generate_requests_to_time(osrm, T)
        print(self)
        self.assign(osrm, T)
        
    def assign(self, osrm, T):
        l = len(self.queue)
        for i in range(l):
            req = self.queue.popleft()
            self.insert_heuristics(osrm, req)
            if not self.insert_heuristics(osrm, req):
                pass
                # if (req.Clp <= T)
                #     self.queue.append(req)
        
    def insert_heuristics(self, osrm, req):
        dc = np.inf
        v_ = None
        r_ = None
        viol = None
        for v in self.vehs:
            route = []
            for leg in v.route:
                route.append( (leg.rid, leg.pod, leg.tlng, leg.tlat) )
            l = len(route)
            c = v.c
            for i in range(l+1):
                for j in range(i+1, l+2):
                    route.insert(i, (req.id, 1, req.olng, req.olat) )
                    route.insert(j, (req.id, -1, req.dlng, req.dlat) )
                    flag, dc_, viol = self.test_constraints_get_cost(osrm, route, v, dc)
                    if flag:
                        dc = dc_
                        v_ = v
                        r_ = copy.deepcopy(route)
                    elif "late_pickup" in viol or "late_dropoff" in viol or "over_capacity" in viol:
                        break
                    route.pop(j)
                    route.pop(i)
                if "late_pickup" in viol:
                    break
        if v_ != None:
            v_.build_route(osrm, r_)
            return True
        else:
            return False
    
    def test_constraints_get_cost(self, osrm, route, v, dc):
        C = v.c + dc
        c = 0.0
        t = 0.0
        n = v.n
        T = v.T
        K = v.K
        lng = v.lng
        lat = v.lat
        for (rid, pod, tlng, tlat) in route:
            dt = osrm.get_duration(lng, lat, tlng, tlat)
            t += dt
            if pod == 1 and T + t > self.reqs[rid].Clp:
                return False, None, ["late_pickup"]
            elif pod == -1 and T + t > self.reqs[rid].Cld:
                return False, None, ["late_dropoff"]
            c += n * dt * B_V
            n += pod
            if n > K:
                return False, None, ["over_capacity"]
            c += t * B_W if pod == 1 else 0
            if c > C:
                return False, None, ["high_cost"]
            lng = tlng
            lat = tlat
        assert c-v.c >= 0
        return True, c-v.c, []
    
    def draw(self):
        for v in self.vehs:
            v.draw()
        for r in self.queue:
            r.draw()
        
    def __str__(self):
        str = "AMoD system: %d vehicles of capacity %d; %.1f trips/h" % (self.V, self.K, 3600/self.D)
        str += "\n  at t = %.3f, %d requests, in which %d in queue" % ( self.T, self.N-1, len(self.queue) )
        # for r in self.queue:
        #     str += "\n" + r.__str__()
        return str