import time
import csv
import copy
import pandas as pd
import argparse
import numpy as np
import random
import datetime
from multiprocessing import *
import mplleaflet
import matplotlib.pyplot as plt
from collections import deque

from lib.OsrmEngine import *
from lib.Agents import *
from lib.Demand import *
from lib.Constants import *

def print_results(model, runtime, now):
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
	print("Scenario: %s; Simulated Annealing: %s; Rebalancing: %s" % (DEMAND_STR, "yes" if IS_SA else "no", "yes" if IS_RB else "no"))
	print("Simulation starts at %s" % (now))
	print("Runtime Time: %d s" % (runtime))
	print("System Settings:")
	print("  - Simulation Time: %d s" % SIMULATION)
	print("  - Fleet Size: %d; Capacity: %d" % (model.V, model.K))
	print("  - Demand Rate: %.1f trips/h" % (model.D))
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
	print("    + vehicle rebalancing time percentage: %.1f%%" % (100.0*vrtt/SIMULATION))
	print("*"*80)

if __name__ == "__main__":
	exe_loc = './osrm-backend-5.6.0/build/osrm-routed'
	map_loc = './osrm-backend-5.6.0/greater-london-latest.osrm'

	osrm = OsrmEngine(exe_loc, map_loc)
	osrm.start_server()
	osrm = OsrmEngine(exe_loc, map_loc)
	osrm.start_server()

	INPUTS = [ 
				[DEMAND4, TOTAL4, "4", 80], 
				[DEMAND4, TOTAL4, "4", 100],
				[DEMAND4, TOTAL4, "4", 120],
				[DEMAND35, TOTAL35, "35", 100],
				[DEMAND35, TOTAL35, "35", 120],
				[DEMAND35, TOTAL35, "35", 140]]
	for DEMAND, TOTAL_DEMAND, DEMAND_STR, FLEET_SIZE in INPUTS:
		now = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M")
		f1 = open("%s_D%s_V%d_K%d_SA%d_RB%d_reqs" % (now, DEMAND_STR, FLEET_SIZE, CAPACITY, 1 if IS_SA else 0, 1 if IS_RB else 0), 'w')
		f2 = open("%s_D%s_V%d_K%d_SA%d_RB%d_vehs" % (now, DEMAND_STR, FLEET_SIZE, CAPACITY, 1 if IS_SA else 0, 1 if IS_RB else 0), 'w')
		w1 = csv.writer(f1)
		w2 = csv.writer(f2)

		model = Model(TOTAL_DEMAND * DEMAND_SCALER, V=FLEET_SIZE, K=CAPACITY)

		stime = time.time()
		for T in range(0,WARM_UP+SIMULATION+WRAP_UP,INTERVAL):
			model.dispatch_at_time(osrm, T)
			if T >= WARM_UP and T <= WARM_UP+SIMULATION:
				w2.writerow([T])
				lats = []
				lngs = []
				idles = []
				rebls = []
				Dss = []
				Tss = []
				Drs = []
				Trs = []
				for veh in model.vehs:
					lats.append(veh.lat)
					lngs.append(veh.lng)
					idles.append(veh.idle)
					rebls.append(veh.rebl)
					Dss.append(veh.Ds)
					Tss.append(veh.Ts)
					Drs.append(veh.Dr)
					Trs.append(veh.Tr)
				w2.writerow(lats)
				w2.writerow(lngs)
				w2.writerow(idles)
				w2.writerow(rebls)
				w2.writerow(Dss)
				w2.writerow(Tss)
				w2.writerow(Drs)
				w2.writerow(Trs)
		etime = time.time()
		runtime = etime - stime

		for req in model.reqs:
			if req.Tr >= WARM_UP and req.Tr <= WARM_UP+SIMULATION:
				w1.writerow([req.id, req.olat, req.olng, req.dlat, req.dlng, req.Tr, req.Tp, req.Td])

		print_results(model, runtime, now)

		f1.close()
		f2.close()