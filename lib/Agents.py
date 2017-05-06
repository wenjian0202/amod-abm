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
        d: total distance 
        t: total duration
        steps: a list of steps
    """
    def __init__(self, d=0.0, t=0.0, steps=[]):
        self.d = d
        self.t = t
        self.steps = deque([])
        
    def add_step(self, step):
        self.steps.append(step)
        
    def __str__(self):
        return "leg: distance = %.1f, duration = %.1f, number of steps = %d" % (self.d, self.t, len(self.steps) )
    
     