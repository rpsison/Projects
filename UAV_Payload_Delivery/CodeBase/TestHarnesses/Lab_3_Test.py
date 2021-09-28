"""At the moment, this test harness is incomplete, only used to demonstrate
that use_wind mode in VehicleAerodynamicsModel.py doesn't affect anything important"""

from TestTools import TestTools as tt

import math
import numpy as np
import itertools
import random
import time


import sys
sys.path.append("..") #python is horrible, no?
from ece163.Modeling import VehicleDynamicsModel as VDM
from ece163.Modeling import VehicleAerodynamicsModel as VAM
from ece163.Modeling import WindModel as WM
from ece163.Controls import VehicleTrim as VT
from ece163.Controls import VehiclePerturbationModels as VPM

from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Utilities import Rotations

lab_name = "ECE163_Lab3"


parser = tt.parse_args_for_lab(lab_name)
args = parser.parse_args()
# args.generate = True #helpful for debugging, leave in for now

#make our testManager
tm = tt.TestManager(lab_name, args)


#%%
tm.start_block("windModel createTransferFcns() tests")
def drydenTransferFcns_constructor_procedure(inputs):
	#first we need a windmodel
	testWM = WM.WindModel(
			Va= inputs["Va"],
			drydenParamters=inputs["drydenParams"])
	
	tfs = testWM.getDrydenTransferFns()
	ret_dict = { 
		key:value for key, value in 
	   zip(["Phi_u", "Gamma_u", "H_u", "Phi_v", "Gamma_v", "H_v", "Phi_w", "Gamma_w", "H_w"],
			tfs)}
	return ret_dict

drydenParams_to_test=[
	"DrydenLowAltitudeLight",
	"DrydenLowAltitudeModerate",
	"DrydenHighAltitudeLight",
	"DrydenHighAltitudeModerate",
	"DrydenNoGusts",
	"DrydenNoWind"
	]
Vas_to_test = [
	25,
	100,
	.001]


for i, (Va, dryden_set) in enumerate(itertools.product(
		Vas_to_test, drydenParams_to_test)):
# 	desc1 = "-".join(state.keys())
	inputs = {"drydenParams":VPC.__dict__[dryden_set], "Va":Va}
	tm.test(f"wm_createTransferFunctions_{i}_{Va}_{dryden_set}", drydenTransferFcns_constructor_procedure,
		 inputs)
	
tm.end_block()

#%%
tm.start_block("windModel update() tests")
def windModel_update_1_step_procedure(inputs):
	#first we need a windmodel
	testWM = WM.WindModel()
	
	input_dict = {"Wn":0, "We":0, "Wd":0}
	for k in ["Wn", "We", "Wd", "drydenParamters"]:
		if k in inputs.keys():
			input_dict[k] = inputs[k]
	
	testWM.setWindModelParameters(**input_dict)
	
	input_dict = {"uu":0, "uv":0, "uw":0}
	for k in ["uu","uv","uw"]:
		if k in inputs.keys():
			input_dict[k] = inputs[k]
	
	testWM.Update(**input_dict)
	
	wind = testWM.getWind()
	
	ret_dict = {k:wind.__dict__[k] for k in ["Wn", "We", "Wd", "Wu", "Wv", "Ww"]}
	return ret_dict

drydenParams_to_test=[
	"DrydenNoWind",
	"DrydenNoGusts",
	"DrydenLowAltitudeLight",
	"DrydenHighAltitudeModerate",
	]
static_winds_to_test=[
	{},
	{"Wn":10},
	{"Wn":30, "We":20, "Wd":10}
	]
u_inputs_to_test=[
	{},
	{"uu":.1},
	{"uv":.1},
	{"uw":.1},
	{"uu":.1, "uv":.1, "uw":.1},
	{"uu":-.1, "uv":-.1, "uw":-.1},	
	]


for i, (wind, dryden_set,u) in enumerate(itertools.product(
		static_winds_to_test, drydenParams_to_test, u_inputs_to_test)):
	desc1 = "-".join(wind.keys())
	desc2 = "-".join(u.keys())
	inputs = {"drydenParamters":VPC.__dict__[dryden_set]}
	inputs.update(wind)
	inputs.update(u)
	tm.test(f"wm_update1_{i}_{desc1}_{dryden_set}_{desc2}", 
		 windModel_update_1_step_procedure,
		 inputs)
	
tm.end_block()


#%%
update_count = 10000
tm.start_block(f"windModel update()x{update_count} tests")
tt.ttprint(tt.SUMMARY, 
"""   (please note that tests in this section are statistical 
   and will fail occasionally with correct code)""")
def windModel_update_multi_step_procedure(inputs):
	#seeding here keeps failures from depending on subset of tests run
	random.seed(time.time())
	
	#first we need a windmodel
	testWM = WM.WindModel()
	
	input_dict = {"Wn":0, "We":0, "Wd":0}
	for k in ["Wn", "We", "Wd", "drydenParamters"]:
		if k in inputs.keys():
			input_dict[k] = inputs[k]
	
	testWM.setWindModelParameters(**input_dict)
	
	#we want the generator to be more repeatable:
	gen_update_count = update_count * (10 if args.generate else 1)
	
	windresults = np.zeros([6,gen_update_count])
	for j in range(gen_update_count):
		testWM.Update()
		wind = testWM.getWind()
		
		for i,v in enumerate(wind.__dict__.values()):
			windresults[i][j] = v
	stds = np.std(windresults, 1)
	means = np.mean(windresults, 1)
	
	ret_dict = {}
	for i,k in enumerate(["Wn", "We", "Wd", "Wu", "Wv", "Ww"]):
		ret_dict["std_"+k] = stds[i]
		ret_dict["mean_"+k] = means[i]
	
	return ret_dict

drydenParams_to_test=[
# 	"DrydenNoWind",  #are these helpful at all here?
 	"DrydenNoGusts",
	"DrydenLowAltitudeLight",
	"DrydenHighAltitudeModerate",
	]
static_winds_to_test=[
# 	{},
	{"Wn":30, "We":20, "Wd":10}
	]


for i, (wind, dryden_set) in enumerate(itertools.product(
		static_winds_to_test, drydenParams_to_test)):
	desc1 = "-".join(wind.keys())
	inputs = {"drydenParamters":VPC.__dict__[dryden_set]}
	inputs.update(wind)
	tm.test(f"wm_update{update_count}_{i}_{desc1}_{dryden_set}",
		  windModel_update_multi_step_procedure,
		 inputs,
		abs_tol= 0.2, rel_tol = 1)
	
tm.end_block()

#%%
tm.start_block("VehicleAerodynamicsModel CaluclateAirspeed() tests")
def calculateAirspeed_procedure(inputs):

	#first we need a VAM and a state
	testState = States.vehicleState(**inputs["state"])
	testWind = States.windState(**inputs["wind"])
	testVAM = VAM.VehicleAerodynamicsModel()
	
	Va, alpha, beta = testVAM.CalculateAirspeed(testState, testWind)
	
	return {"Va":Va, "alpha":alpha, "beta":beta}

states_to_test=[
	{},
	{"u":5},
	{"v":5},
	{"w":5},
	{"v":-10},
	{"u":15, "v":15, "w":15},
	{"u":-3, "v":-3, "w":-3},
	{"yaw":tt.d2r(40)},
	{"pitch":tt.d2r(40)},
	{"roll":tt.d2r(40)},
	{"yaw":tt.d2r(40), "pitch":tt.d2r(-40), "roll":tt.d2r(-20)},
	#maybe some gimbal lock-y stuff:
	{"pitch":tt.d2r(90), "p":3, "q":2, "r":-1},
	{"pitch":tt.d2r(-90), "u":3, "q":2, "r":-1},
	#and at least one where we just mess with everything
	{"u":3, "v":12,
	  "yaw":tt.d2r(30), "pitch":tt.d2r(-20), "roll":tt.d2r(-30) }
	]
static_winds_to_test=[
	{},
	{"Wn":15},
	{"We":15},
	{"Wd":15},
	{"Wn":15,"We":15,"Wd":15},
	]
gusts_to_test = [
	{"Wu":15},
	{"Wv":15},
	{"Ww":15},
	{"Wu":15,"Wv":15,"Ww":15},
	]

for i, (state, static_wind, gust) in enumerate(itertools.product(
		states_to_test, static_winds_to_test, gusts_to_test)):
	wind = {**static_wind, **gust}
	desc1 = "-".join(state.keys())
	desc2 = "-".join(wind.keys())
	inputs = {"state":state, "wind":wind}
	tm.test(f"vam_CalculateAirspeed_{i}_{desc1}_{desc2}", 
		 calculateAirspeed_procedure,
		 inputs)
	
tm.end_block()


#%% Another statistical test, this time for VAM.Update() with wind
update_count = 10000
tm.start_block(f"VAM update()x{update_count} tests")
tt.ttprint(tt.SUMMARY, 
"""   (please note that tests in this section are statistical 
   and will fail occasionally with correct code)""")
def VAM_update_multi_step_procedure(inputs):
	#seeding here keeps failures from depending on subset of tests run
	random.seed(time.time())
	
	#first we need a windmodel
	testVAM = VAM.VehicleAerodynamicsModel()
	testControls = Inputs.controlInputs()
	
	input_dict = {"Wn":0, "We":0, "Wd":0}
	for k in ["Wn", "We", "Wd", "drydenParamters"]:
		if k in inputs.keys():
			input_dict[k] = inputs[k]
	testVAM.getWindModel().setWindModelParameters(**input_dict)
	
	#we want the generator to be more repeatable:
	gen_update_count = update_count * (10 if args.generate else 1)
	
	stateresults = np.zeros([12,gen_update_count])
	for j in range(gen_update_count):
		testVAM.Update(testControls)
		state = testVAM.getVehicleState()
		
		for i,k in enumerate(["pn","pe","pd","u","v","w","yaw","pitch","roll","p","q","r"]):
			stateresults[i][j] = state.__dict__[k]
			
	stds = np.std(stateresults, 1)
	means = np.mean(stateresults, 1)
	
	ret_dict = {}
	for i,k in enumerate(["pn","pe","pd","u","v","w","yaw","pitch","roll","p","q","r"]):
		ret_dict["std_"+k] = stds[i]
		ret_dict["mean_"+k] = means[i]
	
	return ret_dict

drydenParams_to_test=[
# 	"DrydenNoWind",  #are these helpful at all here?
 	# "DrydenNoGusts",
	"DrydenLowAltitudeLight",
	"DrydenHighAltitudeModerate",
	]
static_winds_to_test=[
# 	{},
	{"Wn":30, "We":20, "Wd":10}
	]


for i, (wind, dryden_set) in enumerate(itertools.product(
		static_winds_to_test, drydenParams_to_test)):
	desc1 = "-".join(wind.keys())
	inputs = {"drydenParamters":VPC.__dict__[dryden_set]}
	inputs.update(wind)
	tm.test(f"vam_update{update_count}_{i}_{desc1}_{dryden_set}",
		  VAM_update_multi_step_procedure,
		 inputs,
		rel_tol= 1, abs_tol = 1e-1)
	
tm.end_block()


#%% VehicleTrim tests
#why do we need this?   Well...our VT algorithm relies on some student code,
#for example VehicleDynamicsModel.derivative().  This just checks to make sure
#that they're getting the same as our VehicleTrim results.

tm.start_block("computeTrim() tests")
def computeTrim_procedure(inputs):
	#first we need to make an instance
	testVT = VT.VehicleTrim()
	
	algo_success = testVT.computeTrim(**inputs)
# 	if not algo_success:
# 		raise(Exception(f"Failed to compute trim with inputs {inputs}"))
	ret_dict = {"success":algo_success}
# 	if algo_success:
	ret_dict.update(testVT.getTrimState().__dict__)
	ret_dict.update(testVT.getTrimControls().__dict__)
	
	del ret_dict["chi"] #no idea what's going on here, I'm removing it for now
	
	
	return ret_dict

trim_inputs_to_test=[
 	{},
	{"Vastar":20},
	{"Vastar":35},
	{"Kappastar":0.005},
	{"Kappastar":-0.005},
	{"Gammastar":tt.d2r(20)},
	{"Gammastar":tt.d2r(-30)},
	{"Vastar":20, "Kappastar":0.005, "Gammastar":tt.d2r(20)},
	{"Vastar":35, "Kappastar":-0.005, "Gammastar":tt.d2r(-30)},
	#push it a little bit more:
	{"Vastar":15, "Gammastar":tt.d2r(40)},
	{"Vastar":45, "Gammastar":tt.d2r(-35)},
	#and then a couple of negatives:
	{"Vastar":15},
	{"Vastar":45},
	{"Kappastar":0.05},
	{"Kappastar":-0.05},
	{"Gammastar":tt.d2r(30)},
	{"Gammastar":tt.d2r(-40)},
	]

for i, (trim_inputs) in enumerate(trim_inputs_to_test):
 	desc1 = "-".join(trim_inputs.keys())
 	tm.test(f"computeTrim_{i}_{desc1}",
		  computeTrim_procedure,
 		 trim_inputs,
		  rel_tol = 1e-7)
 	
tm.end_block()

#%% thrust partials tests
tm.start_block("Thrust partial derivative tests")
def dThrust_dX_procedure(inputs):
	
	if "epsilon" in inputs.keys():
		dT_dVa = VPM.dThrust_dVa(inputs["Va"], inputs["Throttle"], 
						   epsilon = inputs["epsilon"])
		dT_dDeltaT = VPM.dThrust_dThrottle(inputs["Va"], inputs["Throttle"],
							epsilon = inputs["epsilon"])
	else:
		dT_dVa = VPM.dThrust_dVa(inputs["Va"], inputs["Throttle"])
		dT_dDeltaT = VPM.dThrust_dThrottle(inputs["Va"], inputs["Throttle"])
	return {"dT_dVa":dT_dVa, "dT_dDeltaT":dT_dDeltaT}

dThrust_inputs=[
 	{"Va":25, "Throttle":0.5},
 	{"Va":25, "Throttle":0.5, "epsilon":.3},
 	{"Va":25, "Throttle":0.5, "epsilon":.01},
 	{"Va":25, "Throttle":0.0},
 	{"Va":25, "Throttle":1.0},
 	{"Va":15, "Throttle":0.25},
 	{"Va":45, "Throttle":0.25},
 	{"Va":45, "Throttle":0.85},
 	{"Va":15, "Throttle":0.85},
	]

for i, (inputs) in enumerate(dThrust_inputs):
 	desc1 = "-".join(inputs.keys())
 	tm.test(f"CreateTransferFunction_{i}_{desc1}",
		  dThrust_dX_procedure,
 		 inputs)
 	
tm.end_block()
#%% transferFunctions tests
#And now we can use the successful trims from earlier to compute tFs

tm.start_block("CreateTransferFunction() tests")
def CreateTransferFunction_procedure(inputs):
	#first we need to make an instance
	testVT = VT.VehicleTrim()
	
	algo_success = testVT.computeTrim(**inputs)
# 	if not algo_success:
# 		raise(Exception(f"Failed to compute trim with inputs {inputs}"))
	ret_dict = {"success":algo_success}
	
	trimState = testVT.getTrimState()
	trimInputs = testVT.getTrimControls()
	
	tF = VPM.CreateTransferFunction(trimState, trimInputs)
# 	if algo_success:
	ret_dict.update(tF.__dict__)
		
	return ret_dict

trim_inputs_to_test=[
 	{},
	{"Vastar":20},
	{"Vastar":35},
	{"Kappastar":0.005},
	{"Kappastar":-0.005},
	{"Gammastar":tt.d2r(20)},
	{"Gammastar":tt.d2r(-30)},
	{"Vastar":20, "Kappastar":0.005, "Gammastar":tt.d2r(20)},
	{"Vastar":35, "Kappastar":-0.005, "Gammastar":tt.d2r(-30)},
	#push it a little bit more:
	{"Vastar":15, "Gammastar":tt.d2r(40)},
	{"Vastar":45, "Gammastar":tt.d2r(-35)},
	#no negatives wanted here tho
	]

for i, (trim_inputs) in enumerate(trim_inputs_to_test):
	desc1 = "-".join(trim_inputs.keys())
	tm.test(f"CreateTransferFunction_{i}_{desc1}",
		  CreateTransferFunction_procedure,
		 trim_inputs,
		  rel_tol = 1e-7)
	
tm.end_block()


#%% wrap it up by printing out a summary or by pickling our results:
tm.conclude()


