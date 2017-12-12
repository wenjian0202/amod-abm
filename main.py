import time

from lib.Utils import *
from lib.OsrmEngine import *
from lib.Agents import *
from lib.Demand import *
from lib.Constants import *
from lib.ModeChoice import *


if __name__ == "__main__":
	# path of the routing server
	exe_loc = './osrm-backend-5.11.0/build/osrm-routed'
	# path of the road network file that the routing server uses
	map_loc = './osrm-backend-5.11.0/greater-london-latest.osrm'

	# if road network is enabled, initialize the routing server
	# otherwise, use Euclidean distance
	osrm = OsrmEngine(exe_loc, map_loc)
	osrm.start_server()
	osrm.kill_server()
	osrm.start_server()
	osrm.kill_server()
	osrm.start_server()

	f = open('output/results.csv', 'a')
	writer = csv.writer(f)
	writer.writerow([None])
	row = ["ASC", "step", "ASSIGN", "REBL", "T_STUDY", "fleet_size", "capacity", "volume",
	 "service_rate", "count_served", "count_reqs", "service_rate_ond", "count_served_ond", "count_reqs_ond", "service_rate_adv", "count_served_adv", "count_reqs_adv",
	 "wait_time", "wait_time_adj", "wait_time_ond", "wait_time_adv", "in_veh_time", "detour_factor", 
	 "veh_service_dist", "veh_service_time", "veh_service_time_percent", "veh_pickup_dist", "veh_pickup_time", "veh_pickup_time_percent",
	 "veh_rebl_dist", "veh_rebl_time", "veh_rebl_time_percent", "veh_load_by_dist", "veh_load_by_time", 
	 "cost", "benefit", None]
	writer.writerow(row)
	f.close()

	for fleet_size in FLEET_SIZE:
		veh_capacity = VEH_CAPACITY
		wait_time_adj = INI_WAIT
		detour_factor = INI_DETOUR
		demand_matrix = INI_MAT
		asc_avpt = ASC_AVPT

		#iteration
		for step in range(ITER_STEPS):
			# run simulation
			model, step, runtime = run_simulation(osrm, step, demand_matrix, fleet_size, veh_capacity, asc_avpt, wait_time_adj, detour_factor)
			# output the simulation results and save data
			wait_time_adj, detour_factor = print_results(model, step, runtime)