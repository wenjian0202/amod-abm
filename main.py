import time

from lib.Utils import *
from lib.OsrmEngine import *
from lib.Agents import *
from lib.Demand import *
from lib.Constants import *
from lib.Env import *


if __name__ == "__main__":
	# path of the routing server
	exe_loc = './osrm-backend-5.11.0/build/osrm-routed'
	# path of the road network file that the routing server uses
<<<<<<< HEAD
	map_loc = './osrm-backend-5.11.0/boston_massachusetts.osm.pbf'
=======
	map_loc = './osrm-backend-5.11.0/massachusetts-latest.osrm'
>>>>>>> 78a89aadeb6da0b74ae5563b39624bc84db883e7

	# if road network is enabled, initialize the routing server
	# otherwise, use Euclidean distance
	osrm = OsrmEngine(exe_loc, map_loc)
	osrm.start_server()

	# define the environment for the Deep Q Network
	env = RebalancingEnv( Model(DMD_MAT, DMD_VOL, V=FLEET_SIZE, K=VEH_CAPACITY), penalty=-0 )

	# define the DQN sequential structure
	nb_actions = env.action_space.n
	input_shape = (1,) + env.state.shape
	input_dim = env.input_dim

	seq = Sequential()
	seq.add(Flatten(input_shape=input_shape))
	seq.add(Dense(256, activation='relu'))
	seq.add(Dense(nb_actions, activation='linear'))

	# instantiate a DQN and load weights from file
	# dqn is used only when the rebalancing method (MET_REBL) is "dqn"
	memory = SequentialMemory(limit=2000, window_length=1)
	policy = EpsGreedyQPolicy()

	dqn = DQNAgent(model=seq, nb_actions=nb_actions, memory=memory, nb_steps_warmup=100,
				   target_model_update=1e-2, policy=policy, gamma=.80)
	dqn.compile(Adam(lr=0.001, epsilon=0.05, decay=0.0), metrics=['mae'])
	dqn.load_weights('weights/dqn_weights_BAL5_150.h5f')

	for i in range(1):
		for ii in range(1):
			# frames record the states of the AMoD model for animation purpose
			frames = []
			# initialize the AMoD model
			model = Model(DMD_MAT, DMD_VOL, dqn=dqn, V=FLEET_SIZE, K=VEH_CAPACITY, assign=MET_ASSIGN, reopt=MET_REOPT, rebl=MET_REBL)
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
			print_results(model, runtime)
