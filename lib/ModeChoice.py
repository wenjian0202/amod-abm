import numpy as np
import pandas as pd 

'''modelling variables''' 
ASC_BIKE    = -5.01
ASC_BUS = -2.87
ASC_CAR = -2.35
ASC_PnK_RIDE = -3.58
ASC_RAIL    = -3.14
ASC_TAXI    = -5.81
B_BIKE_TT    = -1.07
B_BUS_TT    = -0.37
B_CAR_NUM = 0.751
B_CAR_TT = -1.04
B_COST = -0.144
B_PnK_RIDE_TT = -0.277
B_RAIL_TT    = -0.362
B_TAXI_DIS = 0.439
B_TAXI_TT    = -1.83
B_TRANSITNUMTRANSFERS = -0.452
B_TRANSITWALK_TT = -0.536
B_WALK_TT    = -1.21
MU_TRANSIT = 3.23
BETA_AVPT_CAR_TT = -0.37
BETA_AVPT_PT_TT = -0.362
BETA_AVPT_COST = -0.144

'''variables assumptions''' 
min_cost_avpt = 1.3
sharing_discount = 0.75 
transit_conect_discount = 1.0
taxi_wait_time = 0.3

'''distance conversion'''
km2mile = 1.609344
m2mile = 1609.344

def taxifare_calc(autodis):
	'''
	calculates taxi fare based on the distance of the car trip
	TfL website used - a rough piecewise function was created based on their given values
	''' 
	dist_brkup_1 = min(autodis/1000, 1)
	dist_brkup_2 = max(0, min(autodis/1000-1, 1))
	dist_brkup_3 = max(0, min(autodis/1000-2, 2))
	dist_brkup_4 = max(0, min(autodis/1000-4, 2))
	dist_brkup_5 = max(0, autodis/1000-6)
	taxi_fare = 2.6 + ( 5.1 * dist_brkup_1 + 4.1 * dist_brkup_2 + 3.85 * dist_brkup_3 + 4 * (dist_brkup_4 + dist_brkup_5) ) / km2mile
	return taxi_fare

def transit_utility_calc(transit_utility, MU_TRANSIT):
	if transit_utility == 0:
		b = 0
	else: 
		b = np.exp(MU_TRANSIT*transit_utility)
	return b 

def main_CBD(filename, ASC_AVPT, WAIT_TIME, DETOUR_FACTOR):
	df = pd.read_csv(filename)

	'''data preparation'''
	# divide the expansion factor by 10 to represent 1 year (LTDS is 10 years)
	# df['expan_fac/10'] = df['expan_fac'] / 10
	
	#calculate taxi fare 
	df['taxi_fare'] = df['autodis'].apply(lambda value: taxifare_calc(value))
	
	# add av dist, time columns -- not necessary for cbd trips 
	# df['AV_dist'] = df['autodis']
	# df['AV_time'] = df['autotime']
	# add av cost column
	df['cost_temp_1'] = (0.63 + 0.08 * df['avtime'] / 60 + 0.66 * df['avdis']/m2mile)*sharing_discount
	df['min_cost_avpt'] = min_cost_avpt
	df['cost_temp_2']= df[['cost_temp_1', 'min_cost_avpt']].apply(max, axis=1)
	df['AV+PT_cost'] = df['cost_temp_2'] - transit_conect_discount + df['rail_fare']
	'''end of data preparation'''
	 
	'''define utility equations''' 
	df['car_utility'] = ASC_CAR + B_CAR_TT * df['autotime']/600 + B_CAR_NUM * df['num_cars'] \
						+ B_COST * (df['parking_fare']+ df['autodis'] * 5.2416/(1000*76.165) + df['congcharg'] * 11.5)
	df['walk_utility'] = B_WALK_TT * df['walktime']/600
	df['bike_utility'] = ASC_BIKE + B_BIKE_TT * df['biketime']/600
	df['taxi_utility'] = ASC_TAXI + B_TAXI_TT*(df['autotime']/600+taxi_wait_time) + B_TAXI_DIS * df['autodis'] /1000 +  B_COST *  df['taxi_fare'] # assume 3 min waiting time for taxi 
	df['bus_utility'] = (ASC_BUS + B_BUS_TT * (df['bus_transittime'] - df['bus_transitwalktime'])/600 + B_COST * df['bus_fare'] \
						+ B_TRANSITNUMTRANSFERS * df['bus_transitnumtransfers'] + B_TRANSITWALK_TT * df['bus_transitwalktime']/600 ) * df['av5']
	df['rail_utility'] = (ASC_RAIL + B_RAIL_TT * (df['rail_transittime'] - df['rail_transitwalktime'])/600 + B_COST * df['rail_fare'] \
						+ B_TRANSITNUMTRANSFERS * df['rail_transitnumtransfers'] + B_TRANSITWALK_TT * df['rail_transitwalktime']/600 ) * df['av6']
	df['intermodal_utility'] = (ASC_PnK_RIDE + B_PnK_RIDE_TT * (df['pnk_transittime'] - df['pnk_transitwalktime'])/600 \
							  + B_COST * df['pnk_fare'] + B_TRANSITNUMTRANSFERS * (df['pnk_transitnumtransfers']+1) + B_TRANSITWALK_TT *df['pnk_transitwalktime']/600 ) * df['av8']
	df['AVPT_utility'] = ASC_AVPT + BETA_AVPT_CAR_TT * (DETOUR_FACTOR/600 * df['avtime'] + WAIT_TIME/600) + BETA_AVPT_PT_TT * df['avpttime']/600 \
						+ B_TRANSITNUMTRANSFERS * df['avptNumSteps'] \
						+ BETA_AVPT_COST * df['AV+PT_cost']
	# its num of steps not num of transfers, the extra step required to transfer to AV is accounted for by the num of steps

	'''exponent each utility equation'''
	df['car_utility'] = np.exp(df['car_utility'])
	df['walk_utility'] = np.exp(df['walk_utility'])
	df['bike_utility'] = np.exp(df['bike_utility'])
	df['taxi_utility'] = np.exp(df['taxi_utility'])
	df['bus_utility'] = df['bus_utility'].apply(lambda value: transit_utility_calc(value,MU_TRANSIT))
	df['rail_utility'] = df['rail_utility'].apply(lambda value: transit_utility_calc(value,MU_TRANSIT))
	df['intermodal_utility'] = df['intermodal_utility'].apply(lambda value: transit_utility_calc(value,MU_TRANSIT))
	df['AVPT_utility'] = np.exp(df['AVPT_utility']*MU_TRANSIT)
	
	df['exp_sum']  = np.exp(1/MU_TRANSIT * np.log(df['bus_utility'] + df['rail_utility'] + df['intermodal_utility'] + df['AVPT_utility'])) \
		+ df['car_utility'] + df['walk_utility'] + df['bike_utility'] + df['taxi_utility']
				   
	'''
	calc prob
	'''
	df['sum_transit_utility'] = df['bus_utility'] + df['rail_utility'] + df['intermodal_utility'] + df['AVPT_utility']
	df['prob_car'] = df['car_utility']/df['exp_sum']
	df['prob_walk'] = df['walk_utility']/df['exp_sum']
	df['prob_bike'] = df['bike_utility']/df['exp_sum']
	df['prob_taxi'] = df['taxi_utility']/df['exp_sum']
	df['prob_bus'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
						/df['exp_sum'] * df['bus_utility']/df['sum_transit_utility']
	df['prob_rail'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
						/df['exp_sum'] * df['rail_utility']/df['sum_transit_utility']
	df['prob_intermodal'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
						/df['exp_sum'] * df['intermodal_utility']/df['sum_transit_utility']
	df['prob_AVPT'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
						/df['exp_sum'] * df['AVPT_utility']/df['sum_transit_utility']
	
	df['AVPT_choice'] = df['prob_AVPT'] * df['expan_fac/10']    
	
	df_return = df[['ttrip_id','expan_fac','expan_fac/10','AVPT_choice']]
	# df_return = df[['ttrip_id','oeast','onrth','deast','denrth','expan_fac','expan_fac/10','AVPT_choice']]

	return df_return

def main_intrazonal(filename, ASC_AVPT, WAIT_TIME, DETOUR_FACTOR):
	df = pd.read_csv(filename)
	
	'''data preparation'''
	# divide the expansion factor by 10 to represent 1 year (LTDS is 10 years)
	# df['expan_fac/10'] = df['expan_fac'] / 10
	
	#calculate taxi fare
	df['taxi_fare'] = df['autodis'].apply(lambda value: taxifare_calc(value))
	
	# add av dist, time, cost columns
	df['AV_dist'] = df['autodis']
	df['AV_time'] = df['autotime']
	df['temp'] = (0.63 + 0.08 * df['AV_time'] / 60 + 0.66 * df['AV_dist']/m2mile)*sharing_discount
	df['min_cost_avpt'] = min_cost_avpt
	
	df['AV+PT_cost'] = df[['temp', 'min_cost_avpt']].apply(max, axis=1)
	
	'''define utility equations'''
	df['car_utility'] = ASC_CAR + B_CAR_TT * df['autotime']/600 + B_CAR_NUM * df['num_cars'] \
		+ B_COST * (df['parking_fare']+ df['autodis'] * 5.2416/(1000*76.165) + df['congcharg'] * 11.5)
	df['walk_utility'] = B_WALK_TT * df['walktime']/600
	df['bike_utility'] = ASC_BIKE + B_BIKE_TT * df['biketime']/600
	df['taxi_utility'] = ASC_TAXI + B_TAXI_TT*(df['autotime']/600+taxi_wait_time) + B_TAXI_DIS * df['autodis'] /1000 +  B_COST *  df['taxi_fare'] # assume 3 min waiting time for taxi
	df['bus_utility'] = (ASC_BUS + B_BUS_TT * (df['bus_transittime'] - df['bus_transitwalktime'])/600 + B_COST * df['bus_fare'] \
						 + B_TRANSITNUMTRANSFERS * df['bus_transitnumtransfers'] + B_TRANSITWALK_TT * df['bus_transitwalktime']/600 ) * df['av5']
	df['rail_utility'] = (ASC_RAIL + B_RAIL_TT * (df['rail_transittime'] - df['rail_transitwalktime'])/600 + B_COST * df['rail_fare'] \
					   + B_TRANSITNUMTRANSFERS * df['rail_transitnumtransfers'] + B_TRANSITWALK_TT * df['rail_transitwalktime']/600 ) * df['av6']
	df['intermodal_utility'] = (ASC_PnK_RIDE + B_PnK_RIDE_TT * (df['pnk_transittime'] - df['pnk_transitwalktime'])/600 \
							 + B_COST * df['pnk_fare'] + B_TRANSITNUMTRANSFERS * (df['pnk_transitnumtransfers']+1) + B_TRANSITWALK_TT *df['pnk_transitwalktime']/600 ) * df['av8']
	df['AVPT_utility'] = ASC_AVPT + BETA_AVPT_CAR_TT * (DETOUR_FACTOR/600 * df['AV_time'] + WAIT_TIME/600) + BETA_AVPT_COST * df['AV+PT_cost']

	'''exponent each utility equation'''
	df['car_utility'] = np.exp(df['car_utility'])
	df['walk_utility'] = np.exp(df['walk_utility'])
	df['bike_utility'] = np.exp(df['bike_utility'])
	df['taxi_utility'] = np.exp(df['taxi_utility'])
	df['bus_utility'] = df['bus_utility'].apply(lambda value: transit_utility_calc(value,MU_TRANSIT))
	df['rail_utility'] = df['rail_utility'].apply(lambda value: transit_utility_calc(value,MU_TRANSIT))
	df['intermodal_utility'] = df['intermodal_utility'].apply(lambda value: transit_utility_calc(value,MU_TRANSIT))
	df['AVPT_utility'] = np.exp(df['AVPT_utility']*MU_TRANSIT)

	df['exp_sum']  = np.exp(1/MU_TRANSIT * np.log(df['bus_utility'] + df['rail_utility'] + df['intermodal_utility'] + df['AVPT_utility'])) \
		+ df['car_utility'] + df['walk_utility'] + df['bike_utility'] + df['taxi_utility']

	'''
	calc prob
	'''
	df['sum_transit_utility'] = df['bus_utility'] + df['rail_utility'] + df['intermodal_utility'] + df['AVPT_utility']
	df['prob_car'] = df['car_utility']/df['exp_sum']
	df['prob_walk'] = df['walk_utility']/df['exp_sum']
	df['prob_bike'] = df['bike_utility']/df['exp_sum']
	df['prob_taxi'] = df['taxi_utility']/df['exp_sum']
	df['prob_bus'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
					/df['exp_sum'] * df['bus_utility']/df['sum_transit_utility']
	df['prob_rail'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
					/df['exp_sum'] * df['rail_utility']/df['sum_transit_utility']
	df['prob_intermodal'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
					/df['exp_sum'] * df['intermodal_utility']/df['sum_transit_utility']
	df['prob_AVPT'] = np.exp(1/MU_TRANSIT * np.log(df['sum_transit_utility'])) \
					/df['exp_sum'] * df['AVPT_utility']/df['sum_transit_utility']
	df['AVPT_choice'] = df['prob_AVPT'] * df['expan_fac/10']
	

	df_return = df[['ttrip_id','expan_fac','expan_fac/10','AVPT_choice']]
	# df_return = df[['ttrip_id','oeast','onrth','deast','denrth','expan_fac','expan_fac/10','AVPT_choice']]

	return df_return

def set_avpt_demand(step, demand_matrix, ASC_AVPT, WAIT_TIME, DETOUR_FACTOR):
	'''define file paths for each type of trip being modelled, saved in seperate csvs'''
	filepath_cbd = "lib/ModeData/data_olddata_cbd.csv"
	filepath_intrabus = "lib/ModeData/data_olddata_intrazonalbus.csv"
	filepath_intrarail = "lib/ModeData/data_olddata_intrazonalrail.csv"

	'''generating seperate dataframes for each new type of trips being modelled'''
	df1 = main_CBD(filepath_cbd, ASC_AVPT, WAIT_TIME, DETOUR_FACTOR)
	df2 = main_intrazonal(filepath_intrabus, ASC_AVPT, WAIT_TIME, DETOUR_FACTOR)
	df3 = main_intrazonal(filepath_intrarail, ASC_AVPT, WAIT_TIME, DETOUR_FACTOR)
	df = df1.append(df2.append(df3, ignore_index=True), ignore_index=True)
	total_volume = 0.00
	for idx in range(len(df)):
		demand_matrix[idx][4] = 1/(step+1) * df['AVPT_choice'][idx] / 3 + step/(step+1) * demand_matrix[idx][4]
		total_volume += demand_matrix[idx][4]
	accum_volume = 0.00
	for idx in range(len(df)):
		accum_volume += demand_matrix[idx][4]
		demand_matrix[idx][5] = accum_volume/total_volume

	return demand_matrix, total_volume
	

	
	
	
	
	
	
	
	
