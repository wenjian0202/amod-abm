"""
constants are found here
"""

from lib.Demand import *

# fleet size and vehicle capacity
FLEET_SIZE = 10
VEH_CAPACITY = 4

# demand matrix, demand volume from Demand.py
# ASC and the nickname of the run
ASC = -5.50
DMD_STR = "AVPT" + str(ASC)

# warm-up time, study time and cool-down time of the simulation (in seconds)
T_WARM_UP = 60*2
T_STUDY = 60*2
T_COOL_DOWN = 60*2
T_TOTAL = (T_WARM_UP + T_STUDY + T_COOL_DOWN)
# time before which system gets notified of the in-advance requests
T_ADV_REQ = 60*30

# methods for vehicle-request assignment, reoptimization and rebalancing
# ins = insertion heuristics
# hsa = hybrid simulated annealing
# sar = simple anticipatory rebalancing, orp = optimal rebalancing problem, dqn = deep Q network
MET_ASSIGN = "ins"
MET_REOPT = "no"
MET_REBL = "orp"

# intervals for vehicle-request assignment, reoptimization and rebalancing
INT_ASSIGN = 30
INT_REOPT = 30
INT_REBL = 150

# if road network is enabled, use the routing server; otherwise use Euclidean distance
IS_ROAD_ENABLED = True
# if true, activate the animation
IS_ANIMATION = True

# maximum detour factor and maximum wait time window
MAX_DETOUR = 1.5
MAX_WAIT = 60*10

# constant vehicle speed when road network is disabled (in meters/second)
CST_SPEED = 6

# coefficients for wait time and in-vehicle travel time in the utility function
COEF_WAIT = 1.5
COEF_INVEH = 1.0

# map width and height
MAP_WIDTH = 5.52
MAP_HEIGHT = 6.63

# coordinates
# (Olng, Olat) lower left corner
Olng = -0.02
Olat = 51.29
# (Dlng, Dlat) upper right corner
Dlng = 0.18
Dlat = 51.44
# number of cells in the gridded map
Nlng = 10
Nlat = 10
# number of moving cells centered around the vehicle
Mlng = 5
Mlat = 5
# length of edges of a cell
Elng = 0.02
Elat = 0.015
