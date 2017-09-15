"""
constants are found here
"""

from lib.Demand import *

# fleet size and vehicle capacity
FLEET_SIZE = 6
VEH_CAPACITY = 4

# demand matrix, demand volume and its nickname
DMD_MAT = M_AVPT550
DMD_VOL = D_AVPT550 / 10
DMD_STR = "AVPT550"

# warm-up time, study time and cool-down time of the simulation (in seconds)
T_WARM_UP = 60*30
T_STUDY = 60*60
T_COOL_DOWN = 60*30
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
IS_ROAD_ENABLED = False
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
MAP_WIDTH = 4.65
MAP_HEIGHT = 4.23

# coordinates
# (Olng, Olat) lower left corner
Olng = -71.13
Olat = 42.345
# (Dlng, Dlat) upper right corner
Dlng = -71.07
Dlat = 42.385
# number of cells in the gridded map
Nlng = 10
Nlat = 10
# number of moving cells centered around the vehicle
Mlng = 5
Mlat = 5
# length of edges of a cell
Elng = 0.006
Elat = 0.004
