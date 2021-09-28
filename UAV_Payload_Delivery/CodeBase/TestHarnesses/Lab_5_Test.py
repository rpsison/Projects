from TestTools import TestTools as tt

import math
import numpy as np
import itertools
import random
import time
import re


import sys
sys.path.append("..") #python is horrible, no?
from ece163.Modeling import VehicleDynamicsModel as VDM
from ece163.Modeling import VehicleAerodynamicsModel as VAM
from ece163.Modeling import WindModel as WM
from ece163.Controls import VehicleTrim as VT
from ece163.Controls import VehiclePerturbationModels as VPM
from ece163.Controls import VehicleControlGains as VCG
from ece163.Controls import VehicleClosedLoopControl as VCLC 
from ece163.Sensors import SensorsModel as SM

from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Constants import VehicleSensorConstants as VSC
from ece163.Containers import States
from ece163.Containers import Sensors
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import Rotations

lab_name = "ECE163_Lab5"


parser = tt.parse_args_for_lab(lab_name)
args = parser.parse_args()
# args.generate = True #helpful for debugging, leave in for now
# args.tests_to_run = "Noisy"

#make our testManager
tm = tt.TestManager(lab_name, args)



#%% GaussMarkovXYZ 1 step
tm.start_block("GaussMarkovXYZ single update tests")
def GaussMarkovXYZ_update2_procedure(inputs):
	
	testGM = SM.GaussMarkovXYZ(**inputs["GM_params"])
	res_1 = testGM.update(**inputs["noise"])
	res_2 = testGM.update(**inputs["noise"])
	results = {}
	for i, axis in enumerate(["x","y","z"]):
		results[f"{axis}_t1"] = res_1[i]
		results[f"{axis}_t2"] = res_2[i]
	
	return results


gm_params_to_test = [
	{"tauX":1e2, "etaX":1.0, "tauY":1e3, "etaY":3.0, "tauZ":1e1, "etaZ":0.25},
	{"tauX":1e2, "etaX":1.0, "tauY":1e3, "etaY":3.0},
	{"tauX":1e2, "etaX":1.0},
	]

noises_to_test = [
	{"vXnoise":-3, "vYnoise":0, "vZnoise":0},
	{"vXnoise":0, "vYnoise":4, "vZnoise":0},
	{"vXnoise":0, "vYnoise":0, "vZnoise":5},
	{"vXnoise":-3, "vYnoise":4, "vZnoise":5},
	]

for i, (gm, noise) in enumerate(itertools.product(
		gm_params_to_test, noises_to_test )):
	desc1 = "-".join([k[-1] for k in gm.keys()])
	desc2 = "-".join([str(v) for v in noise.values()])
	inputs = {"GM_params":gm, "noise":noise}
	tm.test(f"GaussMarkovXYZ_update2_{i}_{desc1}_{desc2}", GaussMarkovXYZ_update2_procedure,
		 inputs)

tm.end_block()

#%% gaussMarkov 1D multi step
# from matplotlib import pyplot as plt
update_count = 100000
tm.start_block(f"GaussMarkov 1D Update{update_count} tests")
tt.ttprint(tt.SUMMARY, 
"""   (please note that tests in this section are statistical 
   and will fail occasionally with correct code,
    sometimes even generating 'nan' results)""")
def GaussMarkov_1D_update_multi_step_procedure(inputs):
	if args.generate:
		return inputs

	#seeding here keeps failures from depending on subset of tests run
	random.seed(time.time())
	testGM = SM.GaussMarkov(**inputs)
	
	gm_results = np.zeros([update_count])
	for j in range(update_count):
		gm_results[j] = testGM.update()

	#now do an autocorrelation: (GM autocorr should be sigma**2 * exp(-T/tau))
	cutoff = int(update_count*.98) 
	corr = np.correlate(
		gm_results,
		gm_results[0:cutoff],
		mode="valid"  #valid mode only keeps a tiny window - good, that's where our exponential autocorr is the cleanest
		)*testGM.dT**2
	
	#ok let's do a linear regression
	b = np.reshape(corr, (-1, 1)) #b is now column matrix
	b = np.log(b)  
	A = np.ones( [len(corr), 2] ) 
	A[:,1] = [testGM.dT*i for i in range(len(corr))] #A now encodes line formula
# 	x, _, _, _, = np.linalg.lstsq(A, np.log(b))

	x = np.linalg.inv(A.T.dot(A)).dot(A.T).dot(b)
	
	intercept = x[0,0]
	slope = x[1,0]
	
	sigma = np.exp(intercept/2)
	tau = -1/slope
	
# 	print(f"tau={tau}, sigma={sigma}, ")
	
# 	plt.subplots(3,1, sharex=True)
# 	plt.subplot(3,1,1)	
# 	plt.plot([i*testGM.dT for i in range(len(gm_results))],gm_results)
# 	plt.stem([0,cutoff*testGM.dT], [1,1])
# 	
# 	plt.subplot(3,1,2)
# 	plt.plot([i*testGM.dT for i in range(len(corr))], corr)
# 	plt.plot([i*testGM.dT for i in range(len(corr))],
# 		   [sigma**2 * np.exp(- testGM.dT*i/tau) for i in range(len(corr))],)
# 	
# 	
# 	plt.subplot(3,1,3)
# 	plt.plot([i*testGM.dT for i in range(len(corr))], np.log(np.abs(corr)))
# 	plt.plot([i*testGM.dT for i in range(len(corr))],
# 		   [intercept + slope * testGM.dT*i for i in range(len(corr))],)
# 	
	return {"eta":sigma*testGM.dT, "tau":tau}

# plt.close("all")

gm_params_to_test = [
	{"tau":1e3, "eta":.01},
	{"tau":1e2, "eta":.1},
 	{"tau":1e2, "eta":10},
 	{"tau":1e1, "eta":.01},
	]



for i,  gm_params in enumerate(gm_params_to_test):
	desc1 = "-".join([f"{v:e}" for v in gm_params.values()])
	tm.test(f"GaussMarkov_1D_update{update_count}_{i}_{desc1}",
		  GaussMarkov_1D_update_multi_step_procedure,
		 gm_params,
# 		abs_tol= 0.1,
		rel_tol = 1,
		)
tm.end_block()

#%% gaussMarkovXYZ multi step
update_count = 10000
tm.start_block(f"GaussMarkovXYZ Update{update_count} tests")
tt.ttprint(tt.SUMMARY, 
"""   (please note that tests in this section are statistical 
   and will fail occasionally with correct code)""")
def GaussMarkovXYZ_update_multi_step_procedure(inputs):
	#seeding here keeps failures from depending on subset of tests run
	random.seed(time.time())
	
	testGM = SM.GaussMarkovXYZ(**inputs)
	
	#we want the generator to be more repeatable:
	gen_update_count = update_count * (10 if args.generate else 1)
	
	gm_results = np.zeros([3,gen_update_count])
	for j in range(gen_update_count):
		gm_results[:,j] = testGM.update()
		
	stds = np.std(gm_results, 1)
# 	means = np.mean(gm_results, 1)
	
	ret_dict = {}
	for i,k in enumerate(["x","y","z"]):
		ret_dict["std_"+k] = stds[i]
# 		ret_dict["mean_"+k] = means[i]
	
	return ret_dict

gm_params_to_test = [
	{"tauX":1e2, "etaX":1.0},
	{"tauX":1e6, "etaX":.01},
	{"tauX":1e6, "etaX":10.0},
	]


for i,  gm_params in enumerate(gm_params_to_test):
	desc1 = "-".join([f"{v:e}" for v in gm_params.values()])
	tm.test(f"GaussMarkovXYZ_update{update_count}_{i}_{desc1}",
		  GaussMarkovXYZ_update_multi_step_procedure,
		 gm_params,
# 		abs_tol= 0.1,
		rel_tol = 1,
		)
	
	
tm.end_block()


#%%  We can use the same states and dots for many of our inputs (maybe not baro)
test_states = [
	{"pn":10},
	{"pd":10},
	{"pe":10},
	{"pn":-10, "pe":-10, "pd":-10},
	{"u":5},
	{"v":5},
	{"w":5},
	{"u":-3, "v":-3, "w":-3},
	{"p":2},
	{"q":2},
	{"r":2},
	{"p":3, "q":2, "r":-1},
	{"yaw":tt.d2r(40)},
	{"pitch":tt.d2r(40)},
	{"roll":tt.d2r(40)},
	{"yaw":tt.d2r(40), "pitch":tt.d2r(-40), "roll":tt.d2r(-20)},
	#maybe some gimbal lock-y stuff:
	{"pitch":tt.d2r(90), "p":3, "q":2, "r":-1},
	{"pitch":tt.d2r(-90), "u":3, "q":2, "r":-1},
	#and at least one where we just mess with everything
	{"pn":-4, "u":3, "q":2, 
	  "yaw":tt.d2r(30), "pitch":tt.d2r(-20), "roll":tt.d2r(-30) }
	]

test_dots = [
	{},
	{"pn":10},
	{"pd":10},
	{"pe":10},
	{"pn":-10, "pe":-10, "pd":-10},
	{"u":5},
	{"v":5},
	{"w":5},
	{"u":-3, "v":-3, "w":-3},
	{"pn":-1, "pe":-2, "pd":-3, "u":4, "v":5, "w":6},
	{"pn":1, "pe":-2, "pd":3, "u":-4, "v":5, "w":-6},
	]

	
#%% updateAccelsTrue
tm.start_block("updateAccelsTrue tests")
def TestAccelsTrue_procedure(inputs):
	
	testSM = SM.SensorsModel()
	testState = States.vehicleState(**inputs["state"])
	testDot = States.vehicleState(**inputs["dot"])
	y_x, y_y, y_z = testSM.updateAccelsTrue(testState, testDot)
	
	return {"y_x":y_x, "y_y":y_y, "y_z":y_z}

for i, (state, dot) in enumerate(itertools.product(
		test_states, test_dots )):
	desc1 = "-".join(state.keys())
	desc2 = "-".join(dot.keys())
	inputs = {"state":state, "dot":dot}
	tm.test(f"AccelsTrue_{i}_{desc1}_{desc2}", TestAccelsTrue_procedure,
		 inputs)

tm.end_block()

#%% updateGyrosTrue
tm.start_block("updateGyrosTrue tests")
def TestGyrosTrue_procedure(inputs):
	
	testSM = SM.SensorsModel()
	testState = States.vehicleState(**inputs)
	y_x, y_y, y_z = testSM.updateGyrosTrue(testState)
	
	return {"y_x":y_x, "y_y":y_y, "y_z":y_z}

for i,state  in enumerate(test_states):
# 	print(state)
	desc1 = "-".join(state.keys())
	tm.test(f"GyrosTrue_{i}_{desc1}", TestGyrosTrue_procedure,
		 state)

tm.end_block()
	
#%% updateMagsTrue
tm.start_block("updateMagsTrue tests")
def TestMagsTrue_procedure(inputs):
	
	testSM = SM.SensorsModel()
	testState = States.vehicleState(**inputs)
	y_x, y_y, y_z = testSM.updateMagsTrue(testState)
	
	return {"y_x":y_x, "y_y":y_y, "y_z":y_z}

for i,state  in enumerate(test_states):
	desc1 = "-".join(state.keys())
	inputs = {"state":state, "dot":dot}
	tm.test(f"MagsTrue_{i}_{desc1}", TestMagsTrue_procedure,
		 state)

tm.end_block()

#%% updateGPSTrue
tm.start_block("updateGPSTrue tests")
def TestGPSTrue_procedure(inputs):
	
	testSM = SM.SensorsModel()
	testState = States.vehicleState(**inputs["state"])
	testDot = States.vehicleState(**inputs["dot"])
	gps_n, gps_e, gps_alt, gps_sog, gps_cog = testSM.updateGPSTrue(
		testState, testDot)
	
	return {"gps_n":gps_n,  "gps_e":gps_e,  "gps_alt":gps_alt,  "gps_sog":gps_sog,  "gps_cog":gps_cog, }

for i, (state, dot) in enumerate(itertools.product(
		test_states, test_dots )):
	desc1 = "-".join(state.keys())
	desc2 = "-".join(dot.keys())
	inputs = {"state":state, "dot":dot}
	tm.test(f"TestGPSTrue_{i}_{desc1}_{desc2}", TestGPSTrue_procedure,
		 inputs)

tm.end_block()

#%% updatePressureSensorsTrue
tm.start_block("updatePressureSensorsTrue tests")
def TestPressureSensorsTrue_procedure(inputs):
	
	testSM = SM.SensorsModel()
	testState = States.vehicleState(**inputs)
	baro, pitot = testSM.updatePressureSensorsTrue(testState)
	
	return {"baro":baro, "pitot":pitot,}

test_states_pressure = [
	{},
	{"pn":10},
	{"pn":0},
	{"pn":-10},
	{"pn":-100},
	{"pn":-1000},
	{"u":5},
	{"v":5},
	{"w":5},
	{"u":-3, "v":-3, "w":-3},
	]

for i,state  in enumerate(test_states_pressure):
	desc1 = "-".join(state.keys())
	inputs = {"state":state, "dot":dot}
	tm.test(f"PressureSensorsTrue_{i}_{desc1}", TestPressureSensorsTrue_procedure,
		 state)

tm.end_block()


#%% updateSensorsTrue
tm.start_block("updateSensorsTrue tests")
def updateSensorsTrue_procedure(inputs):
	
	
	testSM = SM.SensorsModel(gpsUpdateHz=inputs["gpsUpdateHz"])
	tick_thresh = int(1/testSM.dT/inputs["gpsUpdateHz"])
	prevTrueSensors = Sensors.vehicleSensors()
	
	cur_state = States.vehicleState(**inputs["cur_state"])
	next_state = States.vehicleState(**inputs["cur_state"])
	cur_dot =States.vehicleState(**inputs["cur_dot"])
	next_dot =States.vehicleState(**inputs["cur_state"])
	
	for i in range(tick_thresh-1):
		prevTrueSensors = testSM.updateSensorsTrue(
			prevTrueSensors,  cur_state,  cur_dot)
		
	old = {}
	for key in Sensors.vehicleSensors().__dict__.keys():
		old[key] = prevTrueSensors.__dict__[key]
		
	#ok, we should now ALMOST be at the GPS thresh, so if we give it one more...
	nextTrueSensors = testSM.updateSensorsTrue(
		prevTrueSensors, next_state,  next_dot)
	
	ret = {}
	for key in Sensors.vehicleSensors().__dict__.keys():
		ret["prev_"+key] = old[key]
		ret["next_"+key] = nextTrueSensors.__dict__[key]
	
	
	return ret

gps_rates_to_test = [VSC.GPS_rate,VSC.GPS_rate/2 ]

for i, (j,k, GPS_rate) in enumerate(itertools.product(
		range(len(test_states)), range(len(test_dots)), gps_rates_to_test)):
	
	cur_state = test_states[j]
	next_state = test_states[(j+1)%len(test_states)]
	cur_dot = test_dots[k]
	next_dot = test_dots[(k+1)%len(test_dots)]
	
	desc1 = "-".join(cur_state.keys())
	desc2 = "-".join(cur_dot.keys())
	inputs = {"cur_state":cur_state, "next_state":next_state,
		   "cur_dot":cur_dot, "next_dot":next_dot,
		   "gpsUpdateHz":GPS_rate}
	tm.test(f"updateSensorsTrue_{i}_{desc1}_{desc2}_{GPS_rate}", updateSensorsTrue_procedure,
		 inputs)

tm.end_block()

#%% updateSensorsNoisy
update_count = 1000
tm.start_block("updateSensorsNoisy tests")
tt.ttprint(tt.SUMMARY, 
"""   (please note that tests in this section are statistical 
   and will fail occasionally with correct code)""")
def updateSensorsNoisy_procedure(inputs):
	random.seed(time.time())
	
	testSM = SM.SensorsModel(**inputs["GM_params"])
	biases = Sensors.vehicleSensors(**inputs["biases"])
	sigmas = Sensors.vehicleSensors(**inputs["sigmas"])
	true = Sensors.vehicleSensors(**inputs["true"])

	gen_update_count = update_count * (10 if args.generate else 1)
	
	record = np.zeros([len(Sensors.vehicleSensors().__dict__.items()), gen_update_count])
	prev_reading = Sensors.vehicleSensors()
	
	
	for i in range(gen_update_count):
		y = testSM.updateSensorsNoisy(
			trueSensors=true, 
			noisySensors=prev_reading,
			sensorBiases=biases,
			sensorSigmas=sigmas)
		for j, val in enumerate(y.__dict__.values()):
			record[j,i] = val

	stds = np.std(record, 1)
	means = np.mean(record, 1)
	
	ret_dict = {}
	for i,k in enumerate(Sensors.vehicleSensors().__dict__.keys()):
		ret_dict["std_"+k] = stds[i]
		ret_dict["mean_"+k] = means[i]
	
	
	return ret_dict

gm_params_to_test = [
	{"etagyro":0, "etaGPSHorizontal":0, "etaGPSVertical":0},
	{},
	]
# biases_to_test = []
# sigmas_to_test = []
# trues_to_test = []
trues_to_test = [
	{"gps_sog":10}, #gotta be like this,bad junk happens when your sog is 0
	{"gyro_x":1.0, "gyro_y":2.0, "gyro_z":3.0, "accel_x":4.0, "accel_y":5.0, "accel_z":6.0, "mag_x":7.0, "mag_y":8.0, "mag_z":9.0, "baro":10.0, "pitot":11.0, "gps_n":12.0, "gps_e":13.0, "gps_alt":14.0, "gps_sog":15.0, "gps_cog":16.0}
			  ]
sigmas_biases_to_test = [{},
	{"gyro_x":3.0, "gyro_y":2.0, "gyro_z":1.0},
	{"accel_x":3.0, "accel_y":2.0, "accel_z":1.0},
	{"mag_x":5.0, "mag_y":6.0, "mag_z":3.0,},
	{"baro":2.0, "pitot":1.0},
	{"gps_n":5.0, "gps_e":4.0, "gps_alt":3.0, "gps_sog":2.0, "gps_cog":1.0},
	{"gyro_x":1.0, "gyro_y":2.0, "gyro_z":3.0, "accel_x":4.0, "accel_y":5.0, "accel_z":6.0, "mag_x":7.0, "mag_y":8.0, "mag_z":9.0, "baro":10.0, "pitot":11.0, "gps_n":12.0, "gps_e":13.0, "gps_alt":14.0, "gps_sog":15.0, "gps_cog":16.0}
	]

for i, (gm_params,true, sigmas, biases) in enumerate(itertools.product(
		gm_params_to_test, trues_to_test,
		sigmas_biases_to_test, sigmas_biases_to_test
# 		range(len(sigmas_biases_trues_to_test)) 
	)):
	
# 	desc1 = "-".join([f"{k[0]}{k[-1]}" for k in true.keys()])  #TOO LONG
 	# desc2 = "-".join([f"{k[0:3]}{k[-1]}" for k in gm_params.keys()])
	desc3 = "-".join([f"{k[0]}{k[-1]}" for k in sigmas.keys()])
	desc4 = "-".join([f"{k[0]}{k[-1]}" for k in biases.keys()])
	inputs = {"true":true, "GM_params":gm_params, "sigmas":sigmas, "biases":biases,}
	tm.test(f"updateSensorsNoisy_{i}_{desc3}_{desc4}", updateSensorsNoisy_procedure,
		 inputs,
		 rel_tol = 3)

tm.end_block()

#%% wrap it up by printing out a summary or by pickling our results:
tm.conclude()
