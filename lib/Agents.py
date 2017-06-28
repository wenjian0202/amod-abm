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
    """ 
    def __init__(self, id, rs, K=4, S=6, lng=0.080444, lat=51.381263, T=0.0):
        self.id = id
        self.idle = True
        self.rebl = False
        self.T = T
        if DIRECT:
            self.lat = rs.uniform(-2.5, 2.5)
            self.lng = rs.uniform(-2.5, 2.5)
        else:
            self.lat = lat + rs.normal(0.0, 0.030)
            self.lng = lng + rs.normal(0.0, 0.030) 
        self.tlat = lat
        self.tlng = lng
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
        
    def get_location(self):
        return (self.lng, self.lat)
    
    def get_target_location(self):
        return (self.tlng, self.tlat)
    
    def jump_to_location(self, lng, lat):
        self.lng = lng
        self.lat = lat

    def get_direct_distance(self, lat1, lng1, lat2, lng2):
        # return np.sqrt( (111317 * (lat1-lat2))**2 + (69600 * (lng1-lng2))**2 )
        return np.sqrt( (1000 * (lat1-lat2))**2 + (1000 * (lng1-lng2))**2 )
        
    def build_route(self, osrm, route):
        if len(route) == 0:
            return 
        self.clear_route()
        for (rid, pod, tlng, tlat) in route:
            self.add_leg(osrm, rid, pod, tlng, tlat)
        
    def clear_route(self):
        self.route.clear()
        self.d = 0.0
        self.t = 0.0
        self.c = 0.0
        self.tlng = self.lng
        self.tlat = self.lat
    
    def add_leg(self, osrm, rid, pod, tlng, tlat):
        if DIRECT:
            dis = self.get_direct_distance(self.tlat, self.tlng, tlat, tlng)
            dur = dis / self.S
            leg = Leg(rid, pod, tlng, tlat, dis, dur, steps=[])
            leg.steps.append(Step(dis, dur, [[self.tlng, self.tlat],[tlng, tlat]]))
            self.route.append(leg)
        else:
            out = osrm.get_routing(self.tlng, self.tlat, tlng, tlat)
            assert len(out['legs']) == 1
            leg = Leg(rid, pod, tlng, tlat, 
                      out['legs'][0]['distance'], out['legs'][0]['duration'], steps=[])
            t_leg = 0.0
            for s in out['legs'][0]['steps']:
                step = Step(s['distance'], s['duration'], s['geometry']['coordinates'])
                t_leg += s['duration']
                leg.steps.append(step)
            assert np.isclose(t_leg, leg.t)
            assert len(step.geo) == 2
            assert step.geo[0] == step.geo[1]
            self.route.append(leg)
        self.tlng = leg.steps[-1].geo[1][0]
        self.tlat = leg.steps[-1].geo[1][1]
        self.d += leg.d
        self.t += leg.t
        
    def update_cost_after_move(self, osrm):
        c = 0.0
        t = 0.0
        d = 0.0
        if len(self.route) == 0:
            self.idle = True
            self.rebl = False
            self.c = c
            self.t = t
            self.d = d
            return
        if DIRECT:
            pass
        else:
            out = osrm.get_routing(self.lng, self.lat, self.route[0].tlng, self.route[0].tlat)
            assert len(out['legs']) == 1
            leg = Leg(self.route[0].rid, self.route[0].pod, self.route[0].tlng, self.route[0].tlat, 
                      out['legs'][0]['distance'], out['legs'][0]['duration'], steps=[])
            t_leg = 0.0
            for s in out['legs'][0]['steps']:
                step = Step(s['distance'], s['duration'], s['geometry']['coordinates'])
                t_leg += s['duration']
                leg.steps.append(step)
            assert np.isclose(t_leg, leg.t)
            assert len(step.geo) == 2
            assert step.geo[0] == step.geo[1]
            self.route.popleft()
            self.route.appendleft(leg)
        n = self.n
        for leg in self.route:
            t += leg.t
            d += leg.d
            c += n * leg.t * COEF_INVEH
            n += leg.pod
            c += t * COEF_WAIT if leg.pod == 1 else 0
        assert n == 0
        self.c = c
        self.t = t
        self.d = d
        if self.route[0].rid == -1:
            self.idle = True
            self.rebl = True
            self.c = 0.0
        else:
            self.idle = False
            self.rebl = False 

    def update_cost_after_build(self):
        c = 0.0
        if len(self.route) == 0:
            self.idle = True
            self.rebl = False
            self.c = c
            return
        elif self.route[0].rid == -1:
            self.idle = True
            self.rebl = True
            self.c = c
            return
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
                while dT > 0 and len(leg.steps) > 0:
                    step = leg.steps[0]
                    if step.t < dT:
                        dT -= step.t
                        self.T += step.t
                        if self.T >= WARM_UP and self.T <= WARM_UP+SIMULATION:
                            self.Ts += step.t if leg.rid != -1 else 0
                            self.Ds += step.d if leg.rid != -1 else 0
                            self.Tr += step.t if leg.rid == -1 else 0
                            self.Dr += step.d if leg.rid == -1 else 0
                        self.pop_step()
                        if len(leg.steps) == 0:
                            # corner case: leg.t extremely small, but still larger than dT
                            self.jump_to_location(leg.tlng, leg.tlat)
                            self.n += leg.pod
                            done.append( (leg.rid, leg.pod, self.T) )
                            self.pop_leg()
                            break
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
                        self.cut_fake_step(step, pct)
                        lng = step.geo[0][0]
                        lat = step.geo[0][1]
                        return lng, lat, n
        assert dT > 0 or np.isclose(dT, 0.0)
        assert len(route) == 0 
        assert n == 0
        return lng, lat, n
        
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

    def cut_fake_step(self, step, pct):
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
        if DIRECT:
            self.Clp = np.inf
            self.Cld = np.inf
        else:
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
        DEMAND: demand matrix
        D: arrival rate (trips/hour)
        V: number of vehicles
        K: capacity of vehicles
        vehs: the list of vehicles
        N: number of requests
        reqs: the list of requests
        queue: requests in the queue
        rs1: a seeded random generator for requests
        rs2: a seeded random generator for vehicle locations
    """ 
    def __init__(self, DEMAND, D, dqn=None, V=2, K=4):
        self.rs1 = np.random.RandomState(np.random.randint(0,1000000))
        self.rs2 = np.random.RandomState(np.random.randint(0,1000000))
        self.T = 0.0
        self.DEMAND = DEMAND
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
        
    def generate_request(self, osrm):
        dt = 3600.0/self.D * self.rs1.exponential()
        rand = self.rs1.rand()
        for d in self.DEMAND:
            if d[5] > rand:
                req = Req(osrm, 
                          0 if self.N == 0 else self.reqs[-1].id+1,
                          dt if self.N == 0 else self.reqs[-1].Tr+dt,
                          d[0], d[1], d[2], d[3])
                break
        return req

    def generate_request_random_seed(self, osrm):
        dt = 3600.0/self.D * np.random.exponential()
        rand = np.random.rand()
        for d in self.DEMAND:
            if d[5] > rand:
                req = Req(osrm, -1, -1, d[0], d[1], d[2], d[3])
                break
        return req
        
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
        
    def dispatch_at_time(self, osrm, T):
        self.T = T
        for veh in self.vehs:
            done = veh.move_to_time(T)
            for (rid, pod, t) in done:
                if pod == 1:
                    self.reqs[rid].Tp = t
                elif pod == -1:
                    self.reqs[rid].Td = t
            veh.update_cost_after_move(osrm)
        self.generate_requests_to_time(osrm, T)
        print(self)
        self.assign(osrm, T)
        if T % REBL_INT == 0:
            if REBALANCE == "sar":
                self.rebalance_sar(osrm)
            elif REBALANCE == "orp":
                self.rebalance_orp(osrm, T)    
            elif REBALANCE == "dqn" and self.dqn != None:
                self.rebalance_dqn(osrm)    
        
    def assign(self, osrm, T):
        l = len(self.queue)
        for i in range(l):
            req = self.queue.popleft()
            if not self.insert_heuristics(osrm, req):
                self.rejs.append(req)
        if SIMU_ANNEAL == "yes":
            if T >= WARM_UP:
                self.simulated_annealing(osrm)

    def rebalance_sar(self, osrm):
        Nx = 5
        Ny = 5
        Ex = 0.02
        Ey = 0.015
        for veh in self.vehs:
            if veh.idle:
                veh.clear_route()
                veh.rebl = False
                [d, v, s], center = self.get_state(veh, Nx, Ny, Ex, Ey)
                n = np.random.uniform(0, np.sum(d))
                m = 0
                for i,j in itertools.product(range(Ny), range(Nx)):
                    m += d[i][j]
                    if m > n:
                        break
                route = [(-1, 0, center[i][j][0], center[i][j][1])]
                veh.build_route(osrm, route)
                veh.update_cost_after_build()

    def rebalance_orp(self, osrm, T):
        Wx = 0.2
        Wy = 0.15
        Nx = 10
        Ny = 10
        Ex = Wx/Nx
        Ey = Wy/Ny
        d = np.zeros((Ny,Nx))
        c = np.zeros((Ny,Nx,2))
        v = np.zeros((Ny,Nx))
        s = np.zeros((Ny,Nx))
        b = np.zeros((Ny,Nx))
        for dmd in self.DEMAND:
            for i,j in itertools.product(range(Ny), range(Nx)):
                if dmd[0] >= 51.44 - (i+1)*Ey:
                    if dmd[1] <= -0.02 + (j+1)*Ex:
                        d[i][j] += dmd[4] * self.D
                        c[i][j][0] += dmd[0] * dmd[4] * self.D
                        c[i][j][1] += dmd[1] * dmd[4] * self.D
                        break
        for i,j in itertools.product(range(Ny), range(Nx)):
            if d[i][j] != 0:
                c[i][j][0] /= d[i][j]
                c[i][j][1] /= d[i][j]
        for veh in self.vehs:
            if veh.idle:
                veh.clear_route()
                veh.rebl = False
                for i,j in itertools.product(range(Ny), range(Nx)):
                    if veh.lat >= 51.44 - (i+1)*Ey:
                        if veh.lng <= -0.02 + (j+1)*Ex:
                            v[i][j] += 1
                            break
            else:
                lng, lat, n = veh.get_location_at_time(T+REBL_INT)
                for i,j in itertools.product(range(Ny), range(Nx)):
                    if lat >= 51.44 - (i+1)*Ey:
                        if lng <= -0.02 + (j+1)*Ex:
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
        for i,j in itertools.product(range(Ny), range(Nx)):
            if d[i][j] == 0:
                continue
            lamda = d[i][j] * REBL_INT/3600
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
                    if DIRECT:
                        dis_ = veh.get_direct_distance(veh.lat, veh.lng, c[i][j][0], c[i][j][1])
                    else:
                        dis_ = osrm.get_distance(veh.lng, veh.lat, c[i][j][1], c[i][j][0])
                    if dis_ < dis:
                        dis = dis_
                        vid = vid_
            route = [(-1, 0, c[i][j][0], c[i][j][1])]
            self.vehs[vid].build_route(osrm, route)
            self.vehs[vid].update_cost_after_build()
            for i_, j_ in itertools.product(range(Ny), range(Nx)):
                if self.vehs[vid].lat >= 51.44 - (i_+1)*Ey:
                    if self.vehs[vid].lng <= -0.02 + (j_+1)*Ex:
                        v[i_][j_] -= 1
                        break
            s[i][j] += 1
            if d[i][j] == 0:
                continue
            lamda = d[i][j] * REBL_INT/3600
            k = 0
            p = 0.0
            while k <= s[i][j]:
                p += np.exp(-lamda) * (lamda**k) / np.math.factorial(k)
                k += 1
            b[i][j] = 1 - p
        assert np.sum(v) == 0
        assert np.min(v) == 0

    def rebalance_dqn(self, osrm):
        Nx = 5
        Ny = 5
        Ex = 0.02
        Ey = 0.015
        for veh in self.vehs:
            if veh.idle:
                veh.clear_route()
                veh.rebl = False
                state, center = self.get_state(veh, Nx, Ny, Ex, Ey)
                action = self.dqn.forward(state)
                self.act(osrm, veh, action, center, Nx, Ny, Ex, Ey)

    def get_state(self, veh, Nx, Ny, Ex, Ey):
        lng = veh.lng
        lat = veh.lat
        d = np.zeros((Ny,Nx))
        c = np.zeros((Ny,Nx,2))
        v = np.zeros((Ny,Nx))
        s = np.zeros((Ny,Nx))
        for dmd in self.DEMAND:
            for i,j in itertools.product(range(Ny), range(Nx)):
                if dmd[0] <= lat + Ny*Ey/2 - i*Ey and dmd[0] >= lat + Ny*Ey/2 - (i+1)*Ey:
                    if dmd[1] >= lng - Nx*Ex/2 + j*Ex and dmd[1] <= lng - Nx*Ex/2 + (j+1)*Ex:
                        d[i][j] += dmd[4] * self.D
                        c[i][j][0] += dmd[0] * dmd[4] * self.D
                        c[i][j][1] += dmd[1] * dmd[4] * self.D
                        break
        for i,j in itertools.product(range(Ny), range(Nx)):
            if d[i][j] != 0:
                c[i][j][0] /= d[i][j]
                c[i][j][1] /= d[i][j]
            else:
                c[i][j][0] = False
                c[i][j][1] = False
        for veh_ in self.vehs:
            if veh_.idle:
                for i,j in itertools.product(range(Ny), range(Nx)):
                    if veh_.lat <= lat + Ny*Ey/2 - i*Ey and veh_.lat >= lat + Ny*Ey/2 - (i+1)*Ey:
                        if veh_.lng >= lng - Nx*Ex/2 + j*Ex and veh_.lng <= lng - Nx*Ex/2 + (j+1)*Ex:
                            v[i][j] += 1
                            break
            else:
                lng_, lat_, n = veh_.get_location_at_time(self.T+REBL_INT)
                for i,j in itertools.product(range(Ny), range(Ny)):
                    if lat_ <= lat + Ny*Ey/2 - i*Ey and lat_ >= lat + Ny*Ey/2 - (i+1)*Ey:
                        if lng_ >= lng - Nx*Ex/2 + j*Ex and lng_ <= lng - Nx*Ex/2 + (j+1)*Ex:
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

    def act(self, osrm, veh, action, c, Nx, Ny, Ex, Ey):
        assert veh.idle
        i = int((Ny-1)/2)
        j = int((Nx-1)/2)
        lng = veh.lng
        lat = veh.lat
        if action == 0:
            if c[i][j][0]:
                lat = c[i][j][0]
                lng = c[i][j][1]
            else:
                action = np.random.randint(1, 9)
        if action == 1:
            if c[i-1][j+1][0]:
                lat = c[i-1][j+1][0]
                lng = c[i-1][j+1][1]
            elif c[i-2][j+2][0]:
                lat = c[i-2][j+2][0]
                lng = c[i-2][j+2][1]
        elif action == 2:
            if c[i][j+1][0]:
                lat = c[i][j+1][0]
                lng = c[i][j+1][1]
            elif c[i][j+2][0]:
                lat = c[i][j+2][0]
                lng = c[i][j+2][1]
        elif action == 3:
            if c[i+1][j+1][0]:
                lat = c[i+1][j+1][0]
                lng = c[i+1][j+1][1]
            elif c[i+2][j+2][0]:
                lat = c[i+2][j+2][0]
                lng = c[i+2][j+2][1]  
        elif action == 4:
            if c[i+1][j][0]:
                lat = c[i+1][j][0]
                lng = c[i+1][j][1]
            elif c[i+2][j][0]:
                lat = c[i+2][j][0]
                lng = c[i+2][j][1]
        elif action == 5:
            if c[i+1][j-1][0]:
                lat = c[i+1][j-1][0]
                lng = c[i+1][j-1][1]
            elif c[i+2][j-2][0]:
                lat = c[i+2][j-2][0]
                lng = c[i+2][j-2][1]
        elif action == 6:
            if c[i][j-1][0]:
                lat = c[i][j-1][0]
                lng = c[i][j-1][1]
            elif c[i][j-2][0]:
                lat = c[i][j-2][0]
                lng = c[i][j-2][1]
        elif action == 7:
            if c[i-1][j-1][0]:
                lat = c[i-1][j-1][0]
                lng = c[i-1][j-1][1]
            elif c[i-2][j-2][0]:
                lat = c[i-2][j-2][0]
                lng = c[i-2][j-2][1]
        elif action == 8:
            if c[i-1][j][0]:
                lat = c[i-1][j][0]
                lng = c[i-1][j][1]
            elif c[i-2][j][0]:
                lat = c[i-2][j][0]
                lng = c[i-2][j][1]
        route = [(-1, 0, lng, lat)]
        # print(lng, lat, action, route)
        veh.clear_route()
        veh.rebl = False
        veh.build_route(osrm, route)
        veh.update_cost_after_build()
        
    def insert_heuristics(self, osrm, req):
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
            veh_.build_route(osrm, route_)
            veh_.update_cost_after_build()
            print("    Insertion Heuristics: veh %d is assigned to req %d" % (veh_.id, req.id) )
            return True
        else:
            print("    Insertion Heuristics: req %d is rejected!" % (req.id) )
            return False

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
                veh.update_cost_after_build()
                if not np.isclose(veh.c, route[1]):
                    pass

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
                if viol > 0:
                    break
            if viol == 2:
                break
        if c_ != np.inf:
            rc[0] = route_
            rc[1] = c_
            return True
        else:
            return False

    def get_total_cost(self):
        c = 0.0
        for veh in self.vehs:
            c += veh.c
        return c

    def get_routes_cost(self, routes):
        c = 0.0
        for rc in routes:
            c += rc[1]
        return c
    
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
            dt = 0
            if DIRECT:
                dt = veh.get_direct_distance(lat, lng, tlat, tlng) / veh.S
            else:
                dt = osrm.get_duration(lng, lat, tlng, tlat)
            t += dt
            if pod == 1 and T + t > self.reqs[rid].Clp:
                return False, None, 2 if rid == req.id else 0 # late pickup 
            elif pod == -1 and T + t > self.reqs[rid].Cld:
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