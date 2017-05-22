import time
import csv
import copy
import pandas as pd
import argparse
import numpy as np
import random
from multiprocessing import *
import mplleaflet
import matplotlib.pyplot as plt
from collections import deque

from lib.OsrmEngine import *
from lib.Agents import *
from lib.Demand import *
from lib.Constants import *

def print_results(model, runtime):
    count_reqs = 0
    count_served = 0
    wt = 0.0
    vt = 0.0
    for req in model.reqs:
        if req.Tr >= WARM_UP and req.Tr <= WARM_UP+SIMULATION:
            count_reqs += 1
            if not np.isclose(req.Td, -1.0):
                count_served += 1
                wt += (req.Tp - req.Tr)
                vt += (req.Td - req.Tp)
    wt /= count_served
    vt /= count_served
    
    vstt = 0.0
    vsdt = 0.0
    vrtt = 0.0
    vrdt = 0.0
    for veh in model.vehs:
        vstt += veh.Ts
        vsdt += veh.Ds
        vrtt += veh.Tr
        vrdt += veh.Dr
    vstt /= model.V
    vsdt /= model.V
    vrtt /= model.V
    vrdt /= model.V

    print("*"*80)
    print("Runtime Time: %d s" % (runtime))
    print("System Settings:")
    print("  - Simulation Time: %d s" % SIMULATION)
    print("  - Fleet Size: %d; Capacity: %d" % (model.V, model.K))
    print("  - Demand Rate: %.1f trips/h" % (3600.0/model.D))
    print("  - Dispatching Interval: %.1f s" % INTERVAL)
    print("Simulation Results:")
    print("  - Requests:")
    print("    + service rate: %.1f%% (%d/%d)" % (
        100.0*count_served/count_reqs, count_served, count_reqs))
    print("    + wait time: %.1f s" % (wt))
    print("    + in-vehicle time: %.1f s" % (vt))
    print("  - Vehicles:")
    print("    + vehicle service time travelled: %.1f s" % (vstt))
    print("    + vehicle service distance travelled: %.1f m" % (vsdt))
    print("    + vehicle service time percentage: %.1f%%" % (100.0*vstt/SIMULATION))
    print("    + vehicle rebalancing time travelled: %.1f s" % (vrtt))
    print("    + vehicle rebalancing distance travelled: %.1f m" % (vrdt))
    print("*"*80)

if __name__ == "__main__":
	exe_loc = './osrm-backend-5.6.0/build/osrm-routed'
	map_loc = './osrm-backend-5.6.0/greater-london-latest.osrm'

	osrm = OsrmEngine(exe_loc, map_loc)
	osrm.start_server()
	osrm = OsrmEngine(exe_loc, map_loc)
	osrm.start_server()

	model = Model(20, demand, V=40, K=4)

	stime = time.time()
	for T in range(0,WARM_UP+SIMULATION+WRAP_UP,INTERVAL):
	    model.dispatch_at_time(osrm, T)
	etime = time.time()
	runtime = etime - stime

	print_results(model, runtime)