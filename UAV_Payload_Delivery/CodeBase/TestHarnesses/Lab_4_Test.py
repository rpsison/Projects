"""At the moment, this test harness is incomplete, only used to demonstrate
that use_wind mode in VehicleAerodynamicsModel.py doesn't affect anything important"""

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

from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import Rotations

lab_name = "ECE163_Lab4"


parser = tt.parse_args_for_lab(lab_name)
args = parser.parse_args()
# args.generate = True #helpful for debugging, leave in for now

#make our testManager
tm = tt.TestManager(lab_name, args)


#%% general trim conditions:

if args.generate:	
	# this code is used to generate the dicts below:
	trim_inputs = [
	 	{"Vastar":20},
	 	{"Vastar":35},
	 	{"Vastar":20, "Kappastar":0.005},
	 	{"Vastar":20, "Kappastar":0.005, "Gammastar":tt.d2r(20)},
	 	{"Vastar":15, "Gammastar":tt.d2r(40)},
	 	{"Vastar":45, "Gammastar":tt.d2r(-35)},
	 	]
	
	trimstates =[]
	trimcontrols =[]
	for inputs in trim_inputs:
	 	testVT = VT.VehicleTrim()
	 	algo_success = testVT.computeTrim(**inputs)
	 	assert(algo_success)
	 	trimstates.append(testVT.getTrimState())
	 	trimcontrols.append(testVT.getTrimControls())
	
	tm.store_pregen("trimstates", [ts.__dict__ for ts in trimstates])
	tm.store_pregen("trimcontrols", [tc.__dict__ for tc in trimcontrols])
		 
	#And we want transfer functions that arise from these:
	transferfunctions =[]
	for (state_dict, control_dict) in zip(trimstates, trimcontrols):
		tf = VPM.CreateTransferFunction(state_dict, control_dict)
		transferfunctions.append(tf)
	tm.store_pregen("transferfunctions",[tf.__dict__ for tf in transferfunctions])
else:
	trimstates = tm.retrieve_pregen("trimstates")
	trimcontrols = tm.retrieve_pregen("trimcontrols")
	transferfunctions = tm.retrieve_pregen("transferfunctions")
	


#%% generate some semirandom tunings

# dic = Controls.controlTuning().__dict__
# for i, k in enumerate(dic.keys()):
#  	dic[k] = i+1.0
# print(dic)
# for i, k in enumerate(dic.keys()):
#  	dic[k] = 15.0-i
# print(dic)
# for i, k in enumerate(dic.keys()):
# 	if re.search("Zeta",k):
# 		dic[k] = 1/(15.0-i)
# 	else:
# 		dic[k] = 15.0-i	
# print(tt.round_dict(dic))

#%%
tm.start_block("VehicleControlGains computeGains() tests")
def computeGains_procedure(inputs):
	tf = Linearized.transferFunctions(**inputs["tf"])
	tuning = Controls.controlTuning(**inputs["tunings"])
	gains = VCG.computeGains(tuning, tf)

# 	print("GAINS=",tt.round_dict(gains.__dict__))

	return gains.__dict__

tunings_to_test = [
	{'Wn_roll': 1.0,  'Zeta_roll': 1.0,  'Wn_course': 1.0,  'Zeta_course': 1.0,  'Wn_sideslip': 1.0,  'Zeta_sideslip': 1.0,  'Wn_pitch': 1.0,  'Zeta_pitch': 1.0,  'Wn_altitude': 1.0,  'Zeta_altitude': 1.0,  'Wn_SpeedfromThrottle': 1.0,  'Zeta_SpeedfromThrottle': 1.0,  'Wn_SpeedfromElevator': 1.0,  'Zeta_SpeedfromElevator': 1.0}, 
{'Wn_roll': 1.0, 'Zeta_roll': 2.0, 'Wn_course': 3.0, 'Zeta_course': 4.0, 'Wn_sideslip': 5.0, 'Zeta_sideslip': 6.0, 'Wn_pitch': 7.0, 'Zeta_pitch': 8.0, 'Wn_altitude': 9.0, 'Zeta_altitude': 10.0, 'Wn_SpeedfromThrottle': 11.0, 'Zeta_SpeedfromThrottle': 12.0, 'Wn_SpeedfromElevator': 13.0, 'Zeta_SpeedfromElevator': 14.0}, 
{'Wn_roll': 15.0, 'Zeta_roll': 14.0, 'Wn_course': 13.0, 'Zeta_course': 12.0, 'Wn_sideslip': 11.0, 'Zeta_sideslip': 10.0, 'Wn_pitch': 9.0, 'Zeta_pitch': 8.0, 'Wn_altitude': 7.0, 'Zeta_altitude': 6.0, 'Wn_SpeedfromThrottle': 5.0, 'Zeta_SpeedfromThrottle': 4.0, 'Wn_SpeedfromElevator': 3.0, 'Zeta_SpeedfromElevator': 2.0},
{'Wn_roll': 15.0, 'Zeta_roll': 0.071, 'Wn_course': 13.0, 'Zeta_course': 0.083, 'Wn_sideslip': 11.0, 'Zeta_sideslip': 0.1, 'Wn_pitch': 9.0, 'Zeta_pitch': 0.12, 'Wn_altitude': 7.0, 'Zeta_altitude': 0.17, 'Wn_SpeedfromThrottle': 5.0, 'Zeta_SpeedfromThrottle': 0.25, 'Wn_SpeedfromElevator': 3.0, 'Zeta_SpeedfromElevator': 0.5}
	]

tfs_to_test = tm.retrieve_pregen("transferfunctions")

for i, (tuning, tf) in enumerate(itertools.product(
		tunings_to_test, tfs_to_test)):
# 	desc1 = "-".join(state.keys())
	inputs = {"tunings":tuning, "tf":tf}
	tm.test(f"computeGains_{i}", computeGains_procedure,
		 inputs)
	
tm.end_block()


#%%
tm.start_block("VehicleControlGains computeTuningParameters() tests")
def computeTuningParameters_procedure(inputs):
	tf = Linearized.transferFunctions(**inputs["tf"])
	gains = Controls.controlGains(**inputs["gains"])
	tuning = VCG.computeTuningParameters(gains, tf)

	return tuning.__dict__

gains_to_test = [
	{'kp_roll': 0.012, 'kd_roll': -0.19, 'ki_roll': 0.001, 'kp_sideslip': 11.0, 'ki_sideslip': 8.3, 'kp_course': 4.1, 'ki_course': 2.0, 'kp_pitch': 2.7, 'kd_pitch': 0.097, 'kp_altitude': -0.0016, 'ki_altitude': -0.00079, 'kp_SpeedfromThrottle': 0.27, 'ki_SpeedfromThrottle': 0.15, 'kp_SpeedfromElevator': 0.0029, 'ki_SpeedfromElevator': 0.0016} ,
	{'kp_roll': 3.0, 'kd_roll': 0.04, 'ki_roll': 0.001, 'kp_sideslip': 2.0, 'ki_sideslip': 2.0, 'kp_course': 5.0, 'ki_course': 2.0, 'kp_pitch': -10.0, 'kd_pitch': -0.8, 'kp_altitude': 0.08, 'ki_altitude': 0.03, 'kp_SpeedfromThrottle': 2.0, 'ki_SpeedfromThrottle': 1.0, 'kp_SpeedfromElevator': -0.5, 'ki_SpeedfromElevator': -0.1},
{'kp_roll': 0.53, 'kd_roll': -0.091, 'ki_roll': 0.001, 'kp_sideslip': 3.0, 'ki_sideslip': 450.0, 'kp_course': 9.9, 'ki_course': 780.0, 'kp_pitch': 2.1, 'kd_pitch': 0.062, 'kp_altitude': -0.017, 'ki_altitude': -0.36, 'kp_SpeedfromThrottle': 0.21, 'ki_SpeedfromThrottle': 2.6, 'kp_SpeedfromElevator': 0.086, 'ki_SpeedfromElevator': 0.31},
	]

tfs_to_test = tm.retrieve_pregen("transferfunctions")

for i, (gains, tf) in enumerate(itertools.product(
		gains_to_test, tfs_to_test)):
# 	desc1 = "-".join(state.keys())
	inputs = {"gains":gains, "tf":tf}
	tm.test(f"computeTuningParameters_{i}", computeTuningParameters_procedure,
		 inputs)
	
tm.end_block()

#%% Next let's test the feedback controllers:
	
#%%
tm.start_block("VehicleClosedLoopControl PIDControl tests")
def PIDControl_procedure(inputs):
	
	controller = VCLC.PIDControl()
	controller.setPIDGains(**inputs["controller_params"])
	
	u=0
	for i in range(inputs["steps"]):
		u = controller.Update(0.8, u*.8, 0.1) 
	
	return {"u_final":u}

ks_to_test = [
	{"kp":3.0},
	{"kd":.5},
	{"ki":.7},
	{"kp":3.0, "kd":.5, "ki":.7},
	]
limits_to_test = [
	{"lowLimit":-100.0, "highLimit":100.0},
	{"lowLimit":-1.0, "highLimit":1.0}
	]
trims_to_test = [
	{},
	{"trim":.5},
	]

steps_to_test = [1, 10, 100, 1000]

tfs_to_test = tm.retrieve_pregen("transferfunctions")

for i, (steps, ks, limits, trim) in enumerate(itertools.product(
		steps_to_test, ks_to_test, limits_to_test, trims_to_test )):
	controller_params = {}
	controller_params.update(ks)
	controller_params.update(trim)
	desc1 = "-".join(controller_params.keys())
	controller_params.update(limits)
	inputs = {"controller_params":controller_params, "steps":steps}
	tm.test(f"PIDControl_{i}_n{steps}_{desc1}", PIDControl_procedure,
		 inputs)
	
tm.end_block()


#%%
tm.start_block("VehicleClosedLoopControl PIControl tests")
def PIControl_procedure(inputs):
	
	controller = VCLC.PIControl()
	controller.setPIGains(**inputs["controller_params"])
	
	u=0
	for i in range(inputs["steps"]):
		u = controller.Update(0.8, u*.8) 
	
	return {"u_final":u}

ks_to_test = [
	{"kp":3.0},
	{"ki":.7},
	{"kp":3.0, "ki":.7},
	]
limits_to_test = [
	{"lowLimit":-100.0, "highLimit":100.0},
	{"lowLimit":-1.0, "highLimit":1.0}
	]
trims_to_test = [
	{},
	{"trim":.5},
	]

steps_to_test = [1, 10, 100, 1000]

tfs_to_test = tm.retrieve_pregen("transferfunctions")

for i, (steps, ks, limits, trim) in enumerate(itertools.product(
		steps_to_test, ks_to_test, limits_to_test, trims_to_test )):
	controller_params = {}
	controller_params.update(ks)
	controller_params.update(trim)
	desc1 = "-".join(controller_params.keys())
	controller_params.update(limits)
	inputs = {"controller_params":controller_params, "steps":steps}
	tm.test(f"PIControl_{i}_n{steps}_{desc1}", PIControl_procedure,
		 inputs)
	
tm.end_block()

#%%
tm.start_block("VehicleClosedLoopControl PDControl tests")
def PDControl_procedure(inputs):
	
	controller = VCLC.PDControl()
	controller.setPDGains(**inputs["controller_params"])
	
	u=0
	for i in range(inputs["steps"]):
		u = controller.Update(0.8, u*.8, 0.1) 
	
	return {"u_final":u}

ks_to_test = [
	{"kp":3.0},
	{"kd":.5},
	{"kp":3.0, "kd":.5},
	]
limits_to_test = [
	{"lowLimit":-100.0, "highLimit":100.0},
	{"lowLimit":-1.0, "highLimit":1.0}
	]
trims_to_test = [
	{},
	{"trim":.5},
	]

steps_to_test = [1, 10, 100, 1000]

tfs_to_test = tm.retrieve_pregen("transferfunctions")

for i, (steps, ks, limits, trim) in enumerate(itertools.product(
		steps_to_test, ks_to_test, limits_to_test, trims_to_test )):
	controller_params = {}
	controller_params.update(ks)
	controller_params.update(trim)
	desc1 = "-".join(controller_params.keys())
	controller_params.update(limits)
	inputs = {"controller_params":controller_params, "steps":steps}
	tm.test(f"PDControl_{i}_n{steps}_{desc1}", PDControl_procedure,
		 inputs)
	
tm.end_block()

#%%
tm.start_block("VehicleClosedLoopControl setControlGains tests")
def setControlGains_procedure(inputs):
	
	controlGains = Controls.controlGains(**inputs["controlGains"])
	trimInputs = Inputs.controlInputs(**inputs["trimInputs"])

	vclc = VCLC.VehicleClosedLoopControl()
	vclc.setTrimInputs(trimInputs)
	vclc.setControlGains(controlGains)
	
	ret_dict = {}
	
	#PIs:
	for ctrl in ['rollFromCourse', 'throttleFromAirspeed', 'pitchFromAltitude', 'pitchFromAirspeed', 'rudderFromSideslip']:
		controller = vclc.__dict__[ctrl]
		for key in ["kp", 'ki', 'trim', 'lowLimit', 'highLimit']:
			ret_dict[f"{ctrl}_{key}"] = controller.__dict__[key]
	#PID:
	for ctrl in ['aileronFromRoll']:
		controller = vclc.__dict__[ctrl]
		for key in ["kp", 'kd', 'ki', 'trim', 'lowLimit', 'highLimit']:
			ret_dict[f"{ctrl}_{key}"] = controller.__dict__[key]
	#PD:
	for ctrl in ['elevatorFromPitch']:
		controller = vclc.__dict__[ctrl]
		for key in ["kp", 'kd', 'trim', 'lowLimit', 'highLimit']:
			ret_dict[f"{ctrl}_{key}"] = controller.__dict__[key]
	
	
	return ret_dict

gains_to_test = [
	{'kp_roll': 0.012, 'kd_roll': -0.19, 'ki_roll': 0.001, 'kp_sideslip': 11.0, 'ki_sideslip': 8.3, 'kp_course': 4.1, 'ki_course': 2.0, 'kp_pitch': 2.7, 'kd_pitch': 0.097, 'kp_altitude': -0.0016, 'ki_altitude': -0.00079, 'kp_SpeedfromThrottle': 0.27, 'ki_SpeedfromThrottle': 0.15, 'kp_SpeedfromElevator': 0.0029, 'ki_SpeedfromElevator': 0.0016} ,
 	{'kp_roll': 3.0, 'kd_roll': 0.04, 'ki_roll': 0.001, 'kp_sideslip': 2.0, 'ki_sideslip': 2.0, 'kp_course': 5.0, 'ki_course': 2.0, 'kp_pitch': -10.0, 'kd_pitch': -0.8, 'kp_altitude': 0.08, 'ki_altitude': 0.03, 'kp_SpeedfromThrottle': 2.0, 'ki_SpeedfromThrottle': 1.0, 'kp_SpeedfromElevator': -0.5, 'ki_SpeedfromElevator': -0.1},
	 {'kp_roll': 0.53, 'kd_roll': -0.091, 'ki_roll': 0.001, 'kp_sideslip': 3.0, 'ki_sideslip': 450.0, 'kp_course': 9.9, 'ki_course': 780.0, 'kp_pitch': 2.1, 'kd_pitch': 0.062, 'kp_altitude': -0.017, 'ki_altitude': -0.36, 'kp_SpeedfromThrottle': 0.21, 'ki_SpeedfromThrottle': 2.6, 'kp_SpeedfromElevator': 0.086, 'ki_SpeedfromElevator': 0.31},
	]
trimInputs_to_test = [
	{},
	{'Throttle': 0.5, 'Aileron': tt.d2r(10), 'Elevator': tt.d2r(20), 'Rudder': tt.d2r(30)}
	]

for i, (gains,trims) in enumerate(itertools.product( 
		gains_to_test, trimInputs_to_test)):
	inputs = {"controlGains":gains, "trimInputs":trims}
	tm.test(f"setControlGains_{i}", setControlGains_procedure,
		 inputs)
	
tm.end_block()


#%%
tm.start_block("VehicleClosedLoopControl UpdateControlCommands tests")
def UpdateControlCommands_procedure(inputs):
	
	controlGains = Controls.controlGains(**inputs["controlGains"])
	trimInputs = Inputs.controlInputs(**inputs["trimInputs"])
	referenceCommand = Controls.referenceCommands(**inputs["referenceCommand"])
	initialState = States.vehicleState(**inputs["initialState"])

	vclc = VCLC.VehicleClosedLoopControl()
	vclc.setTrimInputs(trimInputs)
	vclc.setControlGains(controlGains)
	
	outputs = vclc.UpdateControlCommands(referenceCommand, initialState)
	
	ret_dict = {}
	for key in ['Throttle', 'Aileron', 'Elevator', 'Rudder']:
		ret_dict[key] = outputs.__dict__[key]
	for key in ["commandedPitch", "commandedRoll"]:
		ret_dict[key] = referenceCommand.__dict__[key]
	
	return ret_dict

gains_to_test = [
# 	{'kp_roll': 0.012, 'kd_roll': -0.19, 'ki_roll': 0.001, 'kp_sideslip': 11.0, 'ki_sideslip': 8.3, 'kp_course': 4.1, 'ki_course': 2.0, 'kp_pitch': 2.7, 'kd_pitch': 0.097, 'kp_altitude': -0.0016, 'ki_altitude': -0.00079, 'kp_SpeedfromThrottle': 0.27, 'ki_SpeedfromThrottle': 0.15, 'kp_SpeedfromElevator': 0.0029, 'ki_SpeedfromElevator': 0.0016} ,
	{'kp_roll': 3.0, 'kd_roll': 0.04, 'ki_roll': 0.001, 'kp_sideslip': 2.0, 'ki_sideslip': 2.0, 'kp_course': 5.0, 'ki_course': 2.0, 'kp_pitch': -10.0, 'kd_pitch': -0.8, 'kp_altitude': 0.08, 'ki_altitude': 0.03, 'kp_SpeedfromThrottle': 2.0, 'ki_SpeedfromThrottle': 1.0, 'kp_SpeedfromElevator': -0.5, 'ki_SpeedfromElevator': -0.1},
# {'kp_roll': 0.53, 'kd_roll': -0.091, 'ki_roll': 0.001, 'kp_sideslip': 3.0, 'ki_sideslip': 450.0, 'kp_course': 9.9, 'ki_course': 780.0, 'kp_pitch': 2.1, 'kd_pitch': 0.062, 'kp_altitude': -0.017, 'ki_altitude': -0.36, 'kp_SpeedfromThrottle': 0.21, 'ki_SpeedfromThrottle': 2.6, 'kp_SpeedfromElevator': 0.086, 'ki_SpeedfromElevator': 0.31},
	]

trimInputs_to_test = [
	{},
	{'Throttle': 0.5, 'Aileron': tt.d2r(10), 'Elevator': tt.d2r(20), 'Rudder': tt.d2r(30)}
	]


initial_states_to_test = [
	{"v":25},
	{"v":25, "u":10},
	{"v":25, "w":10},
	{"v":25, "u":20, "w":10},
	{"v":55},
	{"v":0},
	{"v":25, "yaw":tt.d2r(30)},
	{"v":25, "pitch":tt.d2r(30)},
	{"v":25, "roll":tt.d2r(30)},
	{"v":25, "pitch":tt.d2r(90)},
	{"v":25, "pitch":tt.d2r(-90)},
	{"v":25, "p":5},
	{"v":25, "q":5},
	{"v":25, "r":5},
	{"v":25, "p":5, "q":5, "r":5},
]

reference_commands_to_test = [
	{},
	{"altitudeCommand":0},
	{"altitudeCommand":-90},
	{"altitudeCommand":-100},
	{"altitudeCommand":-110},
	{"altitudeCommand":-200},
	
	{"courseCommand":-tt.d2r(10)},
	{"courseCommand":tt.d2r(10)},
	{"courseCommand":-math.pi},
	{"courseCommand":math.pi},	
	{"courseCommand":-2*math.pi},
	{"courseCommand":2*math.pi},
	{"courseCommand":-3*math.pi},
	{"courseCommand":3*math.pi},
	
	{"airspeedCommand":10},
	{"airspeedCommand":25},
	{"airspeedCommand":100},
	
	{'courseCommand': tt.d2r(30),  'altitudeCommand': -110.0,  'airspeedCommand': 100} ,	
	{'courseCommand': tt.d2r(30),  'altitudeCommand': -200.0,  'airspeedCommand': 100} ,	
	{'courseCommand': tt.d2r(30),  'altitudeCommand': 0,  'airspeedCommand': 25.0}, 	
	]
	

for i, (controlGains, 	trimInputs,
		referenceCommand, 	initialState	) in enumerate(itertools.product( 
		gains_to_test, 		trimInputs_to_test,
		reference_commands_to_test, 	initial_states_to_test)):
	
	desc1 = "trim" if trimInputs else ""
	desc2 = "-".join(initialState.keys())
	desc3 = "-".join(referenceCommand.keys())
	
	inputs = {"controlGains":controlGains, "trimInputs":trimInputs,
		      "referenceCommand":referenceCommand, "initialState":initialState}
	tm.test(f"UpdateControlCommands_{i}_{desc1}_{desc2}_{desc3}", UpdateControlCommands_procedure,
		 inputs)
	
tm.end_block()



#%%
tm.start_block("VehicleClosedLoopControl Update() tests")
def VCLC_Update_procedure(inputs):
	steps = inputs["steps"]
	controlGains = Controls.controlGains(**inputs["controlGains"])
	trimInputs = Inputs.controlInputs(**inputs["trimInputs"])
	referenceCommand = Controls.referenceCommands(**inputs["referenceCommand"])
	initialState = States.vehicleState(**inputs["initialState"])

	vclc = VCLC.VehicleClosedLoopControl()
	vclc.setVehicleState(initialState)
	vclc.setTrimInputs(trimInputs)
	vclc.setControlGains(controlGains)
	
	for i in range(steps):
		vclc.Update(referenceCommand)		
	
	final_state = vclc.getVehicleState()
	
	ret_dict = {}
	for key in ['pn', 'pe', 'pd', 'u', 'v', 'w', 'yaw', 'pitch', 'roll', 'p', 'q', 'r']:
		ret_dict[key] = final_state.__dict__[key]
	
	return ret_dict

gains_to_test = [
# 	{'kp_roll': 0.012, 'kd_roll': -0.19, 'ki_roll': 0.001, 'kp_sideslip': 11.0, 'ki_sideslip': 8.3, 'kp_course': 4.1, 'ki_course': 2.0, 'kp_pitch': 2.7, 'kd_pitch': 0.097, 'kp_altitude': -0.0016, 'ki_altitude': -0.00079, 'kp_SpeedfromThrottle': 0.27, 'ki_SpeedfromThrottle': 0.15, 'kp_SpeedfromElevator': 0.0029, 'ki_SpeedfromElevator': 0.0016} ,
	{'kp_roll': 3.0, 'kd_roll': 0.04, 'ki_roll': 0.001, 'kp_sideslip': 2.0, 'ki_sideslip': 2.0, 'kp_course': 5.0, 'ki_course': 2.0, 'kp_pitch': -10.0, 'kd_pitch': -0.8, 'kp_altitude': 0.08, 'ki_altitude': 0.03, 'kp_SpeedfromThrottle': 2.0, 'ki_SpeedfromThrottle': 1.0, 'kp_SpeedfromElevator': -0.5, 'ki_SpeedfromElevator': -0.1},
# {'kp_roll': 0.53, 'kd_roll': -0.091, 'ki_roll': 0.001, 'kp_sideslip': 3.0, 'ki_sideslip': 450.0, 'kp_course': 9.9, 'ki_course': 780.0, 'kp_pitch': 2.1, 'kd_pitch': 0.062, 'kp_altitude': -0.017, 'ki_altitude': -0.36, 'kp_SpeedfromThrottle': 0.21, 'ki_SpeedfromThrottle': 2.6, 'kp_SpeedfromElevator': 0.086, 'ki_SpeedfromElevator': 0.31},
	]

trimInputs_to_test = [
	{},
	]


initial_states_to_test = [
	{"v":25},
	{"v":55},
	{"v":0},
]

reference_commands_to_test = [
	{},
	{"altitudeCommand":0},
	{"altitudeCommand":-100},
	{"altitudeCommand":-200},
	
	{"courseCommand":tt.d2r(10)},
	
	{"airspeedCommand":10},
	{"airspeedCommand":100},
	]
	
steps_to_test = [1, 10, 100, 1000]

for i, (steps,
		controlGains, 	trimInputs,
		referenceCommand, 	initialState	) in enumerate(itertools.product( 
		steps_to_test,
		gains_to_test, 		trimInputs_to_test,
		reference_commands_to_test, 	initial_states_to_test)):
	
	desc1 = "trim" if trimInputs else ""
	desc2 = "-".join(initialState.keys())
	desc3 = "-".join(referenceCommand.keys())
	
	inputs = {"controlGains":controlGains, "trimInputs":trimInputs,
		      "referenceCommand":referenceCommand, "initialState":initialState,
			  "steps":steps}
	tm.test(f"VCLC_Update_{i}_n{steps}_{desc1}_{desc2}_{desc3}", VCLC_Update_procedure,
		 inputs)
	
tm.end_block()


#%% wrap it up by printing out a summary or by pickling our results:
tm.conclude()
