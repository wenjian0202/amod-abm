import numpy as np
from collections import deque
import matplotlib.pyplot as plt

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