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

	for veh_capacity in VEH_CAPACITY:
		for fleet_size in FLEET_SIZE:
			wait_time_adj = INI_WAIT
			detour_factor = INI_DETOUR
			demand_matrix = INI_MAT
			demand_volume = 0.00
			for step in range(2):
				demand_matrix, demand_volume = set_avpt_demand(step, demand_matrix, ASC_AVPT, wait_time_adj, detour_factor)

				# frames record the states of the AMoD model for animation purpose
				frames = []
				# initialize the AMoD model
				model = Model(demand_matrix, demand_volume, V=fleet_size, K=veh_capacity, assign=MET_ASSIGN, rebl=MET_REBL)
				# start time
				stime = time.time()
				# dispatch the system for T_TOTAL seconds, at the interval of INT_ASSIGN
				for T in range(0, T_TOTAL, INT_ASSIGN):
					model.dispatch_at_time(osrm, T)
					if IS_ANIMATION:
						frames.append(copy.deepcopy(model.vehs))
				# end time
				etime = time.time()
				# run time of this simulation
				runtime = etime - stime

				# generate, show and save the animation of this simulation
				if IS_ANIMATION:
					anime = anim(frames)
					anime.save('output/anim.mp4', dpi=300, fps=None, extra_args=['-vcodec', 'libx264'])
					plt.show()

				# output the simulation results and save data
				wait_time_adj, detour_factor = print_results(model, step, runtime)
