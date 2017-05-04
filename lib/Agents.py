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
        return "step: distance = {0}, duration = {1}".format(self.d, self.t)
 
    
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
        self.steps = []
        
    def add_step(self, step):
        self.steps.append(step)
        
    def __str__(self):
        return "leg: distance = {0}, duration = {1}, number of steps = {2}".format(self.d, self.t, len(self.steps) )
    
    
