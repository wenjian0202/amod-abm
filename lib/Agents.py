"""
multiple classes for the AMoD system
"""

import numpy as np
import copy
import math
from collections import deque
import matplotlib.pyplot as plt
import itertools

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
    A leg may consists of a series of steps
    Attributes:
        rid: request id (if rebalancing then -1)
        pod: pickup (+1) or dropoff (-1), rebalancing (0)
        tlng: target (end of leg) longitude
        tlat: target (end of leg) latitude
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
        self.steps = deque(steps)
        
    def __str__(self):
        return "leg: distance = %.1f, duration = %.1f, number of steps = %d" % (self.d, self.t, len(self.steps) )
    

class Veh(object):
    """ 
    Veh is a class for vehicles
    Attributes:
        id: sequential unique id
        idle: is idle
        rebl: is rebalancing
        T: system time at current state
        lat: current lngitude
        lng: current longtitude
        tlat: target (end of route) lngitude
        tlng: target (end of route) longtitude
        K: capacity
        S: speed (m/s)
        n: number of passengers on board
        route: a list of legs
        t: total duration of the route
        d: total distance of the route
        c: total cost (generalized time) of the passegners
        Ds: accumulated service distance traveled
        Ts: accumulated service time traveled
        Dr: accumulated rebalancing distance traveled
        Tr: accumulated rebalancing time traveled
        Lt: accumulated load, weighed by service time
        Ld: accumulated load, weighed by service distance
    """ 
    def __init__(self, id, rs, K=4, S=6, T=0.0):
        self.id = id
        self.idle = True
        self.rebl = False
        self.T = T
        self.lng = (Olng+Dlng)/2 + (Dlng-Olng)*rs.uniform(-0.35, 0.35) 
        self.lat = (Olat+Dlat)/2 + (Dlat-Olat)*rs.uniform(-0.35, 0.35)
        self.tlng = self.lng
        self.tlat = self.lat
        self.K = K
        self.S = S
        self.n = 0
        self.route = deque([])
        self.t = 0.0
        self.d = 0.0
        self.c = 0.0
        self.Ds = 0.0
        self.Ts = 0.0
        self.Dr = 0.0
        self.Tr = 0.0
        self.Lt = 0.0
        self.Ld = 0.0
        
    def get_location(self):
        return (self.lng, self.lat)
    
    def get_target_location(self):
        return (self.tlng, self.tlat)
    
    def jump_to_location(self, lng, lat):
        self.lng = lng
        self.lat = lat
    
    # build the route of the vehicle based on a series of quadruples (rid, pod, tlng, tlat)
    # update t, d, c, idle, rebl accordingly
    # rid, pod, tlng, tlat are defined as in class Leg
    def build_route(self, osrm, route, reqs=None, T=None):
        self.clear_route()
        # if the route is null, vehicle is idle
        if len(route) == 0:
            self.idle = True
            self.rebl = False
            self.t = 0.0
            self.d = 0.0
            self.c = 0.0
            return 
        else:
            for (rid, pod, tlng, tlat) in route:
                self.add_leg(osrm, rid, pod, tlng, tlat, reqs, T)
        # if rid is -1, vehicle is rebalancing
        if self.route[0].rid == -1:
            self.idle = True
            self.rebl = True
            self.c = 0.0
            return
        # else, the vehicle is in service to pick up or dropoff
        else:
            c = 0.0
            self.idle = False
            self.rebl = False
            t = 0.0
            n = self.n
            for leg in self.route:
                t += leg.t
                c += n * leg.t * COEF_INVEH
                n += leg.pod
                c += t * COEF_WAIT if leg.pod == 1 else 0
            assert n == 0
            self.c = c
        
    # remove the current route    
    def clear_route(self):
        self.route.clear()
        self.d = 0.0
        self.t = 0.0
        self.c = 0.0
        self.tlng = self.lng
        self.tlat = self.lat
    
    # add a leg based on (rid, pod, tlng, tlat)
    def add_leg(self, osrm, rid, pod, tlng, tlat, reqs, T):
        if IS_ROAD_ENABLED:
            l = osrm.get_routing(self.tlng, self.tlat, tlng, tlat)
            leg = Leg(rid, pod, tlng, tlat, 
                      l['distance'], l['duration'], steps=[])
            t_leg = 0.0
            for s in l['steps']:
                step = Step(s['distance'], s['duration'], s['geometry']['coordinates'])
                t_leg += s['duration']
                leg.steps.append(step)
            assert np.isclose(t_leg, leg.t)
            # the last step of a leg is always of length 2, consisting of 2 identical points as a flag of the end of the leg
            assert len(step.geo) == 2
            assert step.geo[0] == step.geo[1]
            # if pickup and the vehicle arrives in advance, add an extra wait
            if pod == 1:
                if T+self.t+leg.t < reqs[rid].Cep:
                    wait = reqs[rid].Cep - (T+self.t+leg.t)
                    leg.steps[-1].t += wait
                    leg.t += wait
            self.route.append(leg)
        else:
            d_, t_ = osrm.get_distance_duration(self.tlng, self.tlat, tlng, tlat)
            leg = Leg(rid, pod, tlng, tlat, d_, t_, steps=[])
            leg.steps.append(Step(d_, t_, [[self.tlng, self.tlat],[tlng, tlat]]))
            self.route.append(leg)
        self.tlng = leg.steps[-1].geo[1][0]
        self.tlat = leg.steps[-1].geo[1][1]
        self.d += leg.d
        self.t += leg.t
        
    # update the vehicle location as well as the route after moving to time T    
    def move_to_time(self, T):
        dT = T - self.T
        if dT <= 0:
            return []
        # done is a list of finished legs
        done = []
        while dT > 0 and len(self.route) > 0:
            leg = self.route[0]
            # if the first leg could be finished by then
            if leg.t < dT:
                dT -= leg.t
                self.T += leg.t
                if self.T >= T_WARM_UP and self.T <= T_WARM_UP+T_STUDY:
                    self.Ts += leg.t if leg.rid != -1 else 0
                    self.Ds += leg.d if leg.rid != -1 else 0
                    self.Tr += leg.t if leg.rid == -1 else 0
                    self.Dr += leg.d if leg.rid == -1 else 0
                    self.Lt += leg.t * self.n if leg.rid != -1 else 0
                    self.Ld += leg.d * self.n if leg.rid != -1 else 0
                self.jump_to_location(leg.tlng, leg.tlat)
                self.n += leg.pod
                done.append( (leg.rid, leg.pod, self.T) )
                self.pop_leg()
            else:
                while dT > 0 and len(leg.steps) > 0:
                    step = leg.steps[0]
                    # if the first leg could not be finished, but the first step of the leg could be finished by then
                    if step.t < dT:
                        dT -= step.t
                        self.T += step.t
                        if self.T >= T_WARM_UP and self.T <= T_WARM_UP+T_STUDY:
                            self.Ts += step.t if leg.rid != -1 else 0
                            self.Ds += step.d if leg.rid != -1 else 0
                            self.Tr += step.t if leg.rid == -1 else 0
                            self.Dr += step.d if leg.rid == -1 else 0
                            self.Lt += step.t * self.n if leg.rid != -1 else 0
                            self.Ld += step.d * self.n if leg.rid != -1 else 0
                        self.jump_to_location(leg.tlng, leg.tlat)
                        self.pop_step()
                        if len(leg.steps) == 0:
                            # corner case: leg.t extremely small, but still larger than dT
                            # this is due to the limited precision of the floating point numbers
                            self.jump_to_location(leg.tlng, leg.tlat)
                            self.n += leg.pod
                            done.append( (leg.rid, leg.pod, self.T) )
                            self.pop_leg()
                            break
                    # the vehicle has to stop somewhere within the step
                    else:
                        pct = dT / step.t
                        if self.T >= T_WARM_UP and self.T <= T_WARM_UP+T_STUDY:
                            self.Ts += dT if leg.rid != -1 else 0
                            self.Ds += step.d * pct if leg.rid != -1 else 0
                            self.Tr += dT if leg.rid == -1 else 0
                            self.Dr += step.d * pct if leg.rid == -1 else 0
                            self.Lt += dT * self.n if leg.rid != -1 else 0
                            self.Ld += step.d * pct * self.n if leg.rid != -1 else 0
                        # find the exact location the vehicle stops and update the step
                        self.cut_step(pct)
                        self.jump_to_location(step.geo[0][0], step.geo[0][1])
                        self.T = T
                        return done
        assert dT > 0 or np.isclose(dT, 0.0)
        assert self.T < T or np.isclose(self.T, T)
        assert len(self.route) == 0 
        assert self.n == 0
        assert np.isclose(self.d, 0.0)
        assert np.isclose(self.t, 0.0)
        self.T = T
        self.d = 0.0
        self.t = 0.0
        return done

    # get the location at time T
    # this function is similar to move_to_time(self, T), but it does not update the route
    def get_location_at_time(self, T):
        dT = T - self.T
        if dT <= 0:
            return self.lng, self.lat, self.n
        lng = self.lng
        lat = self.lat
        n = self.n
        route = copy.deepcopy(self.route)
        while dT > 0 and len(route) > 0:
            leg = route[0]
            if leg.t < dT:
                dT -= leg.t
                lng = leg.tlng
                lat = leg.tlat
                n += leg.pod
                route.popleft()
            else:
                while dT > 0 and len(leg.steps) > 0:
                    step = leg.steps[0]
                    if step.t < dT:
                        dT -= step.t
                        leg.steps.popleft()
                        if len(leg.steps) == 0:
                            # corner case: leg.t extremely small, but still larger than dT
                            lng = leg.tlng
                            lat = leg.tlat
                            n += leg.pod
                            route.popleft()
                            break
                    else:
                        pct = dT / step.t
                        self.cut_temp_step(step, pct)
                        lng = step.geo[0][0]
                        lat = step.geo[0][1]
                        return lng, lat, n
        assert dT > 0 or np.isclose(dT, 0.0)
        assert len(route) == 0 
        assert n == 0
        return lng, lat, n
    
    # pop the first leg from the route list    
    def pop_leg(self):
        leg = self.route.popleft()
        self.d -= leg.d
        self.t -= leg.t
    
    # pop the first step from the first leg    
    def pop_step(self):
        step = self.route[0].steps.popleft()
        self.t -= step.t
        self.d -= step.d
        self.route[0].t -= step.t
        self.route[0].d -= step.d
    
    # find the exact location the vehicle stops and update the step    
    def cut_step(self, pct):
        step = self.route[0].steps[0]
        if step.d == 0:
            _pct = pct
        else:
            dis = 0.0
            sega = step.geo[0]
            for segb in step.geo[1:]:
                dis += np.sqrt( (sega[0] - segb[0])**2 + (sega[1] - segb[1])**2)
                sega = segb
            dis_ = 0.0
            _dis = 0.0
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

    # similar to cut_step(self, pct), but always called by get_location_at_time(self, T)
    def cut_temp_step(self, step, pct):
        if step.d != 0:
            dis = 0.0
            sega = step.geo[0]
            for segb in step.geo[1:]:
                dis += np.sqrt( (sega[0] - segb[0])**2 + (sega[1] - segb[1])**2)
                sega = segb
            dis_ = 0.0
            _dis = 0.0
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
    
    # visualize
    def draw(self):
        color = "0.50"
        if self.id == 0:
            color = "red"
        elif self.id == 1:
            color = "orange"
        elif self.id == 2:
            color = "yellow"
        elif self.id == 3:
            color = "green"
        elif self.id == 4:
            color = "blue"
        plt.plot(self.lng, self.lat, color=color, marker='o', markersize=4, alpha=0.5)
        count = 0
        for leg in self.route:
            count += 1
            plt.plot(leg.tlng, leg.tlat, color=color, 
                     marker='s' if leg.pod == 1 else 'x' if leg.pod == -1 else None, markersize=3, alpha=0.5)
            for step in leg.steps:
                geo = np.transpose( step.geo )
                plt.plot(geo[0], geo[1], color=color, linestyle='-' if count<=1 else '--', alpha=0.5)
                    
    def __str__(self):
        str =  "veh %d at (%.7f, %.7f) when t = %.3f; %s; occupancy = %d/%d" % (
            self.id, self.lng, self.lat, self.T, "rebalancing" if self.rebl else "idle" if self.idle else "in service", self.n, self.K)
        str += "\n  service dist/time: %.1f, %.1f; rebalancing dist/time: %.1f, %.1f" % (
            self.Ds, self.Ts, self.Dr, self.Tr)
        str += "\n  has %d leg(s), dist = %.1f, dura = %.1f，cost = %.1f" % (
            len(self.route), self.d, self.t, self.c)
        for leg in self.route:
            str += "\n    %s req %d at (%.7f, %.7f), dist = %.1f, dura = %.1f" % (
                "pickup" if leg.pod == 1 else "dropoff" if leg.pod == -1 else "rebalancing",
                leg.rid, leg.tlng, leg.tlat, leg.d, leg.t)
        return str
    

class Req(object):
    """ 
    Req is a class for requests
    Attributes:
        id: sequential unique id
        Tr: request time
        olng: origin longtitude
        olat: origin lngitude
        dlng: destination longtitude
        dlat: destination lngitude
        Ts: shortest travel time
        OnD: true if on-demand, false if in-advance
        Cep: constraint - earliest pickup
        Clp: constraint - latest pickup
        Cld: constraint - latest dropoff
        Tp: pickup time
        Td: dropoff time
        D: detour factor
    """
    def __init__(self, osrm, id, Tr, olng=0.115662, olat=51.374282, dlng=0.089282, dlat=51.350675, OnD=True):
        self.id = id
        self.Tr = Tr
        self.olng = olng
        self.olat = olat
        self.dlng = dlng
        self.dlat = dlat
        self.Ts = osrm.get_duration(olng, olat, dlng, dlat)
        self.OnD = OnD
        if self.OnD:
            self.Cep = Tr
            self.Clp = Tr + MAX_WAIT
            self.Cld = None
        else:
            self.Cep = Tr + T_ADV_REQ
            self.Clp = None
            self.Cld = self.Cep + MAX_DETOUR * self.Ts
        self.Tp = -1.0
        self.Td = -1.0
        self.D = 0.0
    
    # return origin
    def get_origin(self):
        return (self.olng, self.olat)
    
    # return destination
    def get_destination(self):
        return (self.dlng, self.dlat)
    
    # visualize
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
        rs1: a seeded random generator for requests
        rs2: a seeded random generator for vehicle locations
        T: system time at current state
        M: demand matrix
        D: demand volume (trips/hour)
        dqn: deep Q network for rebalancing
        V: number of vehicles
        K: capacity of vehicles
        vehs: the list of vehicles
        N: number of requests
        reqs: the list of requests
        rejs: the list of rejected requests
        queue: requests in the queue
        assign: assignment method
        reopt: reoptimization method
        rebl: rebalancing method
    """ 
    def __init__(self, M, D, dqn=None, V=2, K=4, assign="ins", reopt="no", rebl="no"):
        # two random generators, the seed of which could be modified for debug use
        self.rs1 = np.random.RandomState(np.random.randint(0,1000000))
        self.rs2 = np.random.RandomState(np.random.randint(0,1000000))
        self.T = 0.0
        self.M = M
        self.D = D
        self.dqn = dqn
        self.V = V
        self.K = K
        self.vehs = []
        for i in range(V):
            self.vehs.append(Veh(i, self.rs2, K=K))
        self.N = 0
        self.reqs = []
        self.rejs = []
        self.queue = deque([])
        self.assign = assign
        self.reopt = reopt
        self.rebl = rebl
        
    # generate one request, following exponential arrival interval   
    def generate_request(self, osrm):
        dt = 3600.0/self.D * self.rs1.exponential()
        rand = self.rs1.rand()
        for m in self.M:
            if m[6] > rand:
                OnD = True
                if m[2] < 51.35:
                    OnD = False if self.rs1.rand() < 0.5 else True
                req = Req(osrm, 
                          0 if self.N == 0 else self.reqs[-1].id+1,
                          dt if self.N == 0 else self.reqs[-1].Tr+dt,
                          m[1], m[2], m[3], m[4], OnD=OnD)
                break
        return req
    
    # generate requests up to time T, following Poisson process  
    def generate_requests_to_time(self, osrm, T):
        if self.N == 0:
            req = self.generate_request(osrm)
            self.reqs.append(req)
            self.N += 1
        while self.reqs[-1].Tr <= T:
            req = self.generate_request(osrm)
            self.queue.append(self.reqs[-1])
            self.reqs.append(req)
            self.N += 1
        assert self.N == len(self.reqs)
    
    # dispatch the AMoD system: move vehicles, generate requests, assign, reoptimize and rebalance    
    def dispatch_at_time(self, osrm, T):
        self.T = T
        for veh in self.vehs:
            done = veh.move_to_time(T)
            for (rid, pod, t) in done:
                if pod == 1:
                    self.reqs[rid].Tp = t
                elif pod == -1:
                    self.reqs[rid].Td = t
                    self.reqs[rid].D = (self.reqs[rid].Td - self.reqs[rid].Tp)/self.reqs[rid].Ts
        self.generate_requests_to_time(osrm, T)
        print(self)
        if np.isclose(T % INT_ASSIGN, 0):
            if self.assign == "ins":
                self.insertion_heuristics(osrm, T)
        if np.isclose(T % INT_REOPT, 0):
            if self.reopt == "hsa":
                self.simulated_annealing(osrm)
        if np.isclose(T % INT_REBL, 0):
            if self.rebl == "sar":
                self.rebalance_sar(osrm)
            elif self.rebl == "orp":
                self.rebalance_orp(osrm, T)    
            elif self.rebl == "dqn":
                assert self.dqn != None
                self.rebalance_dqn(osrm)    
        
    # insertion heuristics    
    def insertion_heuristics(self, osrm, T):
        l = len(self.queue)
        for i in range(l):
            req = self.queue.popleft()
            if not self.insert_heuristics(osrm, req, T):
                self.rejs.append(req)

    # insert a request using the insertion heuristics method
    def insert_heuristics(self, osrm, req, T):
        dc_ = np.inf
        veh_ = None
        route_ = None
        viol = None
        for veh in self.vehs:
            route = []
            if not veh.idle:
                for leg in veh.route:
                    route.append( (leg.rid, leg.pod, leg.tlng, leg.tlat) )
            else:
                assert veh.c == 0
            l = len(route)
            c = veh.c
            for i in range(l+1):
                for j in range(i+1, l+2):
                    route.insert(i, (req.id, 1, req.olng, req.olat) )
                    route.insert(j, (req.id, -1, req.dlng, req.dlat) )
                    flag, c_, viol = self.test_constraints_get_cost(osrm, route, veh, req, c+dc_)
                    if flag:
                        dc_ = c_ - c
                        veh_ = veh
                        route_ = copy.deepcopy(route)
                    route.pop(j)
                    route.pop(i)
                    if viol > 0:
                        break
                if viol == 2:
                    break
        if veh_ != None:
            veh_.build_route(osrm, route_, self.reqs, T)
            print("    Insertion Heuristics: veh %d is assigned to req %d" % (veh_.id, req.id) )
            return True
        else:
            print("    Insertion Heuristics: req %d is rejected!" % (req.id) )
            return False

    # simulated annealing
    def simulated_annealing(self, osrm):
        TEMP = 100
        STEPS = 100
        ROUNDS = 10
        success = False
        base_cost = self.get_total_cost()
        routes = []
        for veh in self.vehs:
            route = []
            if not veh.idle:
                for leg in veh.route:
                    route.append( (leg.rid, leg.pod, leg.tlng, leg.tlat) )
            else:
                assert veh.c == 0
            routes.append([route, veh.c])
        best_routes = copy.deepcopy(routes)
        best_cost = base_cost
        for i in range(ROUNDS):
            print("    Simulated Annealing: round %d, max iteration steps = %d" % (i, STEPS))
            for T in np.linspace(TEMP, 0, STEPS, endpoint=False):
                v1, r1 = self.get_random_veh_req(routes)
                v2, r2 = self.get_random_veh_req(routes)
                # print(v1, r1, v2, r2)
                if v1 == v2:
                    continue
                elif r1 == -1 and r2 == -1:
                    continue
                else:
                    rc1 = copy.deepcopy(routes[v1])
                    rc2 = copy.deepcopy(routes[v2])
                    # print(rc1, rc2)
                    if r1 != -1: 
                        self.remove_req_from_veh(osrm, rc1, v1, r1)
                    if r2 != -1: 
                        self.remove_req_from_veh(osrm, rc2, v2, r2)
                    if r1 != -1:
                        if not self.insert_req_to_veh(osrm, rc2, v2, r1):
                            continue
                    if r2 != -1:
                        if not self.insert_req_to_veh(osrm, rc1, v1, r2):
                            continue
                    # print(rc1, rc2)
                    dc = rc1[1] + rc2[1] - routes[v1][1] - routes[v2][1]
                    if dc < 0 or np.random.rand() < math.exp(-dc/T):
                        # if r1 != -1 and r2 != -1:
                        #     print("    SA: swap req %d (veh %d) and req %d (veh %d), dc = %.1f, p = %.3f" % (
                        #         r1, v1, r2, v2, dc, 1.0 if dc<0 else math.exp(-dc/T)))
                        # elif r1 != -1:
                        #     print("    SA: insert req %d (veh %d) to veh %d, dc = %.1f, p = %.3f" % (
                        #         r1, v1, v2, dc, 1.0 if dc<0 else math.exp(-dc/T)))
                        # elif r2 != -1:
                        #     print("    SA: insert req %d (veh %d) to veh %d, dc = %.1f, p = %.3f" % (
                        #         r2, v2, v1, dc, 1.0 if dc<0 else math.exp(-dc/T)))
                        routes[v1] = copy.deepcopy(rc1)
                        routes[v2] = copy.deepcopy(rc2)
                        cost = self.get_routes_cost(routes)
                        # print("    SA: round %d, temperature = %.1f, base cost = %.1f, cost = %.1f" % (
                        #     i, T, base_cost, cost))
                        if cost < best_cost:
                            best_routes = copy.deepcopy(routes)
                            best_cost = cost
                            success = True
                            print("    Simulated Annealing: a better solution is found!")
            routes = copy.deepcopy(best_routes)
            base_cost = best_cost
        if success:
            for veh, route in zip(self.vehs, best_routes):
                veh.build_route(osrm, route[0])
                if not np.isclose(veh.c, route[1]):
                    pass

    # get a random request from a random vehicle
    def get_random_veh_req(self, routes):
        v = np.random.randint(self.V)
        n = 0
        for leg in routes[v][0]:
            if leg[1] == 1:
                n += 1
        if n == 0:
            return v, -1
        r = np.random.randint(n+1)
        if r == n:
            return v, -1
        n_ = -1
        for leg in routes[v][0]:
            if leg[1] == 1:
                n_ += 1
                if n_ == r:
                    return v, leg[0]

    # remove the request from the vehicle
    def remove_req_from_veh(self, osrm, rc, v, r):
        route_ = rc[0]
        p = -1
        d = -1
        for leg, i in zip(route_, range(len(route_))):
            if leg[0] == r and leg[1] == 1:
                p = i
            elif leg[0] == r and leg[1] == -1:
                d = i
        assert p != -1 and d != -1 and p < d
        route_.pop(p)
        route_.pop(d-1)
        c = 0.0
        t = 0.0
        veh = self.vehs[v]
        lng = veh.lng
        lat = veh.lat
        n = veh.n
        for (rid, pod, tlng, tlat) in route_:
            dt = osrm.get_duration(lng, lat, tlng, tlat)
            t += dt
            c += n * dt * COEF_INVEH
            n += pod
            assert n <= veh.K
            c += t * COEF_WAIT if pod == 1 else 0
            lng = tlng
            lat = tlat
        rc[1] = c

    # insert a request to the vehicle
    def insert_req_to_veh(self, osrm, rc, v, r):
        veh = self.vehs[v]
        req = self.reqs[r]
        c_ = np.inf
        viol = None
        route = copy.deepcopy(rc[0])
        l = len(route)
        for i in range(l+1):
            for j in range(i+1, l+2):
                route.insert(i, (req.id, 1, req.olng, req.olat) )
                route.insert(j, (req.id, -1, req.dlng, req.dlat) )
                flag, c, viol = self.test_constraints_get_cost(osrm, route, veh, req, c_)
                if flag:
                    c_ = c
                    route_ = copy.deepcopy(route)
                route.pop(j)
                route.pop(i)
                if viol in [1,2,3]:
                    break
            if viol == 2:
                break
        if c_ != np.inf:
            rc[0] = route_
            rc[1] = c_
            return True
        else:
            return False

    # get the total cost of all vehicles
    def get_total_cost(self):
        c = 0.0
        for veh in self.vehs:
            c += veh.c
        return c

    # get the total cost from input routes
    def get_routes_cost(self, routes):
        c = 0.0
        for rc in routes:
            c += rc[1]
        return c
    
    # test if a route can satisfy all constraints, and if yes, return the cost of the route
    def test_constraints_get_cost(self, osrm, route, veh, req, C):
        c = 0.0
        t = 0.0
        n = veh.n
        T = veh.T
        K = veh.K
        lng = veh.lng
        lat = veh.lat
        for (rid, pod, tlng, tlat) in route:
            n += pod
            if n > K:
                return False, None, 1 # over capacity
        n = veh.n
        for (rid, pod, tlng, tlat) in route:
            req_ = self.reqs[rid]
            dt = osrm.get_duration(lng, lat, tlng, tlat)
            t += dt
            if pod == 1:
                if req_.OnD:
                    if T + t > req_.Clp:
                        return False, None, 2 if rid == req.id else 0 # late pickup
                    else:
                        req_.Cld = T + t + MAX_DETOUR * req_.Ts
                else:
                    if T + t < req_.Cep:
                        dt += req_.Cep - T - t
                        t += req_.Cep - T - t
            elif pod == -1 and T + t > req_.Cld:
                return False, None, 3 if rid == req.id else 0 # late dropoff
            c += n * dt * COEF_INVEH
            n += pod
            assert n <= veh.K
            c += t * COEF_WAIT if pod == 1 else 0
            if c > C:
                return False, None, 0
            lng = tlng
            lat = tlat
        return True, c, -1

    # rebalance using simple anticipatory rebalancing
    def rebalance_sar(self, osrm):
        for veh in self.vehs:
            if veh.idle:
                veh.clear_route()
                veh.rebl = False
                [d, v, s], center = self.get_state(veh)
                n = np.random.uniform(0, np.sum(d))
                m = 0
                for i,j in itertools.product(range(Mlat), range(Mlng)):
                    m += d[i][j]
                    if m > n:
                        break
                route = [(-1, 0, center[i][j][0], center[i][j][1])]
                veh.build_route(osrm, route)

    # rebalance using optimal rebalancing problem
    def rebalance_orp(self, osrm, T):
        d = np.zeros((Nlat, Nlng))
        c = np.zeros((Nlat, Nlng, 2))
        v = np.zeros((Nlat, Nlng))
        s = np.zeros((Nlat, Nlng))
        b = np.zeros((Nlat, Nlng))
        for m in self.M:
            for i,j in itertools.product(range(Nlat), range(Nlng)):
                if m[1] >= Dlat - (i+1)*Elat:
                    if m[0] <= Olng + (j+1)*Elng:
                        d[i][j] += m[4] * self.D
                        c[i][j][0] += m[0] * m[4] * self.D
                        c[i][j][1] += m[1] * m[4] * self.D
                        break
        for i,j in itertools.product(range(Nlat), range(Nlng)):
            if d[i][j] != 0:
                c[i][j][0] /= d[i][j]
                c[i][j][1] /= d[i][j]
        for veh in self.vehs:
            if veh.idle:
                veh.clear_route()
                veh.rebl = False
                for i,j in itertools.product(range(Nlat), range(Nlng)):
                    if veh.lat >= Dlat - (i+1)*Elat:
                        if veh.lng <= Olng + (j+1)*Elng:
                            v[i][j] += 1
                            break
            else:
                lng, lat, n = veh.get_location_at_time(T+INT_REBL)
                for i,j in itertools.product(range(Nlat), range(Nlng)):
                    if lat >= Dlat - (i+1)*Elat:
                        if lng <= Olng + (j+1)*Elng:
                            if n == 0:
                                s[i][j] += 0.8
                            elif n == 1:
                                s[i][j] += 0.4
                            elif n == 2:
                                s[i][j] += 0.2
                            elif n == 3:
                                s[i][j] += 0.1 
                            else:
                                s[i][j] += 0.0
                            break
        for i,j in itertools.product(range(Nlat), range(Nlng)):
            if d[i][j] == 0:
                continue
            lamda = d[i][j] * INT_REBL/3600
            k = 0
            p = 0.0
            while k <= s[i][j]:
                p += np.exp(-lamda) * (lamda**k) / np.math.factorial(k)
                k += 1
            b[i][j] = 1 - p
        while np.sum(v) > 0:
            i, j = np.unravel_index(b.argmax(), b.shape)
            dis = np.inf
            vid = None
            for vid_, veh in enumerate(self.vehs):
                if veh.idle and not veh.rebl:
                    dis_ = osrm.get_distance(veh.lng, veh.lat, c[i][j][0], c[i][j][1])
                    if dis_ < dis:
                        dis = dis_
                        vid = vid_
            route = [(-1, 0, c[i][j][0], c[i][j][1])]
            self.vehs[vid].build_route(osrm, route)
            for i_, j_ in itertools.product(range(Nlat), range(Nlng)):
                if self.vehs[vid].lat >= Dlat - (i_+1)*Elat:
                    if self.vehs[vid].lng <= Olng + (j_+1)*Elng:
                        v[i_][j_] -= 1
                        break
            s[i][j] += 1
            if d[i][j] == 0:
                continue
            lamda = d[i][j] * INT_REBL/3600
            k = 0
            p = 0.0
            while k <= s[i][j]:
                p += np.exp(-lamda) * (lamda**k) / np.math.factorial(k)
                k += 1
            b[i][j] = 1 - p
        assert np.sum(v) == 0
        assert np.min(v) == 0

    # rebalance using deep Q network
    def rebalance_dqn(self, osrm):
        Mlng = 5
        Mlat = 5
        Elng = 0.02
        Elat = 0.015
        for veh in self.vehs:
            if veh.idle:
                veh.clear_route()
                veh.rebl = False
                state, center = self.get_state(veh)
                action = self.dqn.forward(state)
                self.act(osrm, veh, action, center)

    # get the state of a vehicle
    # a state is defined as the predicted demand, the number of vehicles and their locations, occupancy etc around a vehicle
    def get_state(self, veh):
        lng = veh.lng
        lat = veh.lat
        d = np.zeros((Mlat, Mlng))
        c = np.zeros((Mlat, Mlng,2))
        v = np.zeros((Mlat, Mlng))
        s = np.zeros((Mlat, Mlng))
        for m in self.M:
            for i,j in itertools.product(range(Mlat), range(Mlng)):
                if m[1] <= lat + Mlat*Elat/2 - i*Elat and m[1] >= lat + Mlat*Elat/2 - (i+1)*Elat:
                    if m[0] >= lng - Mlng*Elng/2 + j*Elng and m[0] <= lng - Mlng*Elng/2 + (j+1)*Elng:
                        d[i][j] += m[4] * self.D
                        c[i][j][0] += m[0] * m[4] * self.D
                        c[i][j][1] += m[1] * m[4] * self.D
                        break
        for i,j in itertools.product(range(Mlat), range(Mlng)):
            if d[i][j] != 0:
                c[i][j][0] /= d[i][j]
                c[i][j][1] /= d[i][j]
            else:
                c[i][j][0] = False
                c[i][j][1] = False
        for veh_ in self.vehs:
            if veh_.idle:
                for i,j in itertools.product(range(Mlat), range(Mlng)):
                    if veh_.lat <= lat + Mlat*Elat/2 - i*Elat and veh_.lat >= lat + Mlat*Elat/2 - (i+1)*Elat:
                        if veh_.lng >= lng - Mlng*Elng/2 + j*Elng and veh_.lng <= lng - Mlng*Elng/2 + (j+1)*Elng:
                            v[i][j] += 1
                            break
            else:
                lng_, lat_, n = veh_.get_location_at_time(self.T+INT_REBL)
                for i,j in itertools.product(range(Mlat), range(Mlat)):
                    if lat_ <= lat + Mlat*Elat/2 - i*Elat and lat_ >= lat + Mlat*Elat/2 - (i+1)*Elat:
                        if lng_ >= lng - Mlng*Elng/2 + j*Elng and lng_ <= lng - Mlng*Elng/2 + (j+1)*Elng:
                            if n == 0:
                                s[i][j] += 0.8
                            elif n == 1:
                                s[i][j] += 0.4
                            elif n == 2:
                                s[i][j] += 0.2
                            elif n == 3:
                                s[i][j] += 0.1
                            else:
                                s[i][j] += 0.0
                            break
        return [d,v,s], c

    # build the rebalancing route according to the action
    def act(self, osrm, veh, action, c):
        assert veh.idle
        i = int((Mlat-1)/2)
        j = int((Mlng-1)/2)
        lng = veh.lng
        lat = veh.lat
        if action == 0:
            if c[i][j][0]:
                lng = c[i][j][0]
                lat = c[i][j][1]
            else:
                action = np.random.randint(1, 9)
        if action == 1:
            if c[i-1][j+1][0]:
                lng = c[i-1][j+1][0]
                lat = c[i-1][j+1][1]
            elif c[i-2][j+2][0]:
                lng = c[i-2][j+2][0]
                lat = c[i-2][j+2][1]
        elif action == 2:
            if c[i][j+1][0]:
                lng = c[i][j+1][0]
                lat = c[i][j+1][1]
            elif c[i][j+2][0]:
                lng = c[i][j+2][0]
                lat = c[i][j+2][1]
        elif action == 3:
            if c[i+1][j+1][0]:
                lng = c[i+1][j+1][0]
                lat = c[i+1][j+1][1]
            elif c[i+2][j+2][0]:
                lng = c[i+2][j+2][0]
                lat = c[i+2][j+2][1]  
        elif action == 4:
            if c[i+1][j][0]:
                lng = c[i+1][j][0]
                lat = c[i+1][j][1]
            elif c[i+2][j][0]:
                lng = c[i+2][j][0]
                lat = c[i+2][j][1]
        elif action == 5:
            if c[i+1][j-1][0]:
                lng = c[i+1][j-1][0]
                lat = c[i+1][j-1][1]
            elif c[i+2][j-2][0]:
                lng = c[i+2][j-2][0]
                lat = c[i+2][j-2][1]
        elif action == 6:
            if c[i][j-1][0]:
                lng = c[i][j-1][0]
                lat = c[i][j-1][1]
            elif c[i][j-2][0]:
                lng = c[i][j-2][0]
                lat = c[i][j-2][1]
        elif action == 7:
            if c[i-1][j-1][0]:
                lng = c[i-1][j-1][0]
                lat = c[i-1][j-1][1]
            elif c[i-2][j-2][0]:
                lng = c[i-2][j-2][0]
                lat = c[i-2][j-2][1]
        elif action == 8:
            if c[i-1][j][0]:
                lng = c[i-1][j][0]
                lat = c[i-1][j][1]
            elif c[i-2][j][0]:
                lng = c[i-2][j][0]
                lat = c[i-2][j][1]
        route = [(-1, 0, lng, lat)]
        # print(lng, lat, action, route)
        veh.build_route(osrm, route)
    
    # visualize
    def draw(self):
        fig = plt.figure(figsize=(5,6))
        plt.xlim((-0.02,0.18))
        plt.ylim((51.29,51.44))
        for veh in reversed(self.vehs):
            veh.draw()
        for req in self.queue:
            req.draw()
        plt.show()
        
    def __str__(self):
        str = "AMoD system at t = %.3f: %d requests, in which %d in queue" % ( self.T, self.N-1, len(self.queue) )
        # for r in self.queue:
        #     str += "\n" + r.__str__()
        return str