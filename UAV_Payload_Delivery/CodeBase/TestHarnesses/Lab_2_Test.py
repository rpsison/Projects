"""At the moment, this test harness is incomplete, only used to demonstrate
that use_wind mode in VehicleAerodynamicsModel.py doesn't affect anything important"""

from TestTools import TestTools as tt

import math
import itertools

import sys
sys.path.append("..") #python is horrible, no?
from ece163.Modeling import VehicleDynamicsModel as VDM
from ece163.Modeling import VehicleAerodynamicsModel as VAM

from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Utilities import Rotations

lab_name = "ECE163_Lab2"


parser = tt.parse_args_for_lab(lab_name)
args = parser.parse_args()
# args.generate = True #helpful for debugging, leave in for now

#make our testManager
tm = tt.TestManager(lab_name, args)


#%%
tm.start_block("gravityForces() tests")
def gravityForces_procedure(inputs):
	#setup our testing state:
	testState = States.vehicleState(**inputs)
	testVAM = VAM.VehicleAerodynamicsModel()
 		
	#and now the student code does it's thing
	testVAM.getVehicleDynamicsModel().setVehicleState(testState)
	fM = testVAM.gravityForces(testState)
	
	keys_to_check =[
		"Fx", "Fy", "Fz", "Mx", "My", "Mz"]
	return {key:fM.__dict__[key] for key in keys_to_check}

#I think we can reuse freefall test params here
gF_initial_states = [
	{},
	{"yaw":tt.d2r(30)},
	{"pitch":tt.d2r(30)},
	{"roll":tt.d2r(30)},
	{"yaw":tt.d2r(30),"pitch":tt.d2r(30),"roll":tt.d2r(30)},
	#obligatory gimbal lock
	{"yaw":tt.d2r(30),"pitch":tt.d2r(90),"roll":tt.d2r(30)},
	{"yaw":tt.d2r(30),"pitch":tt.d2r(-90),"roll":tt.d2r(30)},
	]

for i, state in enumerate(gF_initial_states):
	desc1 = "-".join(state.keys())
	tm.test(f"gravityForces_{i}_{desc1}", gravityForces_procedure,
		 state)
	
tm.end_block()


#%%
tm.start_block("CalculateCoeff_alpha() tests")
def CCA_procedure(inputs):
	
	testVAM = VAM.VehicleAerodynamicsModel()
	CL_alpha, CD_alpha, CM_alpha = testVAM.CalculateCoeff_alpha(**inputs)
	
	return {"CL_alpha":CL_alpha,
		    "CD_alpha":CD_alpha,
			 "CM_alpha":CM_alpha}

#I think we can reuse freefall test params here
cca_alphas = [60, 30, 10, 5, 1, 0.1, 0, -0.1, -1, -5, -10, -30, -60, -170, -180]


for i, alpha in enumerate(cca_alphas):
	inputs = {"alpha":tt.d2r(alpha)}
	desc1 = str(alpha).replace(".","")
	tm.test(f"CalculateCoeff_alpha_{i}_{desc1}", CCA_procedure,
	 inputs)
	
tm.end_block()


#%%
tm.start_block("aeroForces() tests")
def aeroForces_procedure(inputs):
	#setup our testing state:
	testState = States.vehicleState(**inputs)
	testVAM = VAM.VehicleAerodynamicsModel()

	#and now the student code does it's thing
	testVAM.getVehicleDynamicsModel().setVehicleState(testState)
	fM = testVAM.aeroForces(testState)
	
	keys_to_check =[
		"Fx", "Fy", "Fz", "Mx", "My", "Mz"]
	return {key:fM.__dict__[key] for key in keys_to_check}

#I think we can reuse freefall test params here
aero_initial_v = [
	{"u":5},
	{"v":5},
	{"w":5},
	{"u":-3, "v":-3, "w":-3},
	{"u":3, "v":3, "w":3},
	{}
	]
aero_initial_non_v = [
	{"pn":-10, "pe":-10, "pd":-10},
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

for i, (non_v, v) in enumerate(itertools.product(
		aero_initial_non_v, aero_initial_v)):
	
	desc1 = "-".join(non_v.keys())
	desc2 = "-".join(v.keys())
	state = {**non_v, **v}
	tm.test(f"aeroForces_{i}_{desc1}_{desc2}", aeroForces_procedure, state)
	
tm.end_block()

#%%
tm.start_block("CalculatePropForces() tests")
def CPF_procedure(inputs):
	testVAM = VAM.VehicleAerodynamicsModel()
	Fx, Mx = testVAM.CalculatePropForces(**inputs)
	
	return {"Fx":Fx, "Mx":Mx}

#I think we can reuse freefall test params here
cpf_throttles = [0.5, 1, 0]
cpf_vas = [5, 25, 0, -3]



for i, (Va, Throttle) in enumerate(itertools.product(
		cpf_vas, cpf_throttles)):
	inputs = {"Va":Va, "Throttle":Throttle}
	desc1 = str(inputs["Va"]).replace(".","")
	desc2 = str(inputs["Throttle"])
	tm.test(f"CalculatePropForces_{i}_{desc1}_{desc2}", CPF_procedure,
	 inputs)
	
tm.end_block()


#%%
tm.start_block("controlForces() tests")
def controlForces_procedure(inputs):
	#setup our testing state:
	testState = States.vehicleState(**inputs["initial_state"])
	testVAM = VAM.VehicleAerodynamicsModel()
	testControls = Inputs.controlInputs(**inputs["controls"])
	
	#and now the student code does it's thing
	testVAM.getVehicleDynamicsModel().setVehicleState(testState)
	fM = testVAM.controlForces(testState, testControls)
	
	keys_to_check =[
		"Fx", "Fy", "Fz", "Mx", "My", "Mz"]
	return {key:fM.__dict__[key] for key in keys_to_check}

#I think we can reuse freefall test params here
cf_initial_states = [
	{},
	{"u":5},
	{"v":5},
	{"w":5},
	{"u":-3, "v":-3, "w":-3},
	{"pn":-10, "pe":-10, "pd":-10},
	{"pitch":tt.d2r(90), "roll":tt.d2r(-90)}, #one for gimbal lock
	{"pn":-4, "u":3, "q":2, 
	  "yaw":tt.d2r(30), "pitch":tt.d2r(-20), "roll":tt.d2r(-30) } #one bagel with everything
	]

cf_controls = [
	{},
	{"Aileron":.2},
	{"Rudder":.2},
	{"Elevator":.2},
	{"Throttle":.9},
	{"Throttle":1},

	{"Aileron":-.4, "Rudder":-.5, "Elevator":-.7},
	{"Aileron":-.3, "Rudder":.2, "Elevator":-.2, "Throttle":.9},	]

for i, (state, control) in enumerate(itertools.product(
		cf_initial_states, cf_controls)):
	desc1 = "-".join(state.keys())
	desc2 = "-".join(control.keys())
	tm.test(f"controlForces_{i}_{desc1}_{desc2}", controlForces_procedure,
	 {"initial_state":state, "controls":control})
	
tm.end_block()

#%%
tm.start_block("updateForces() tests")
def updateForces_procedure(inputs):
	#setup our testing state:
	testState = States.vehicleState(**inputs["initial_state"])
	testVAM = VAM.VehicleAerodynamicsModel()
	testControls = Inputs.controlInputs(**inputs["controls"])
	
	#and now the student code does it's thing
	testVAM.getVehicleDynamicsModel().setVehicleState(testState)
	fM = testVAM.updateForces(testState, testControls)
	
	keys_to_check =[
		"Fx", "Fy", "Fz", "Mx", "My", "Mz"]
	return {key:fM.__dict__[key] for key in keys_to_check}

#I think we can reuse freefall test params here
cf_initial_states = [
	{},
	{"u":5},
	{"v":5},
	{"w":5},
	{"u":-3, "v":-3, "w":-3},
	{"pn":-10, "pe":-10, "pd":-10},
	{"pitch":tt.d2r(90), "roll":tt.d2r(-90)}, #one for gimbal lock
	{"pn":-4, "u":3, "q":2, 
	  "yaw":tt.d2r(30), "pitch":tt.d2r(-20), "roll":tt.d2r(-30) } #one bagel with everything
	]

cf_controls = [
	{},
	{"Aileron":.2},
	{"Rudder":.2},
	{"Elevator":.2},
	{"Throttle":.9},
	{"Throttle":1},

	{"Aileron":-.4, "Rudder":-.5, "Elevator":-.7},
	{"Aileron":-.3, "Rudder":.2, "Elevator":-.2, "Throttle":.9},	]

for i, (state, control) in enumerate(itertools.product(
		cf_initial_states, cf_controls)):
	desc1 = "-".join(state.keys())
	desc2 = "-".join(control.keys())
	tm.test(f"updateForces_{i}_{desc1}_{desc2}", updateForces_procedure,
	 {"initial_state":state, "controls":control})
	
tm.end_block()


#%%
tm.start_block("Update tests")
def Update_procedure(inputs, iterations):
# 	tt.ttprint(tt.DEBUG, f"running with inputs {state_vars}")
	
	#setup our testing state:
	testState = States.vehicleState(**inputs["initial_state"])
	testVAM = VAM.VehicleAerodynamicsModel()
	testControls = Inputs.controlInputs(**inputs["controls"])
	
	#and now the student code does its thing
	testVAM.getVehicleDynamicsModel().setVehicleState(testState)
	for i in range(iterations):
		testVAM.Update(testControls)
	newState = testVAM.getVehicleState()
	
	keys_to_check =[
		"pn","pe","pd","u","v","w",
		"p","q","r","yaw","pitch","roll","R"]
	return {key:newState.__dict__[key] for key in keys_to_check}



#I think we can reuse freefall test params here
Update_initial_states = [
	{},
	{"pitch":tt.d2r(90), "roll":tt.d2r(-90)}, #one for gimbal lock
	{"pn":-4, "u":3, "q":2, 
	  "yaw":tt.d2r(30), "pitch":tt.d2r(-20), "roll":tt.d2r(-30) } #one bagel with everything
	]

Update_controls = [
	{},
	{"Aileron":.2},
	{"Rudder":.2},
	{"Elevator":.2},
	{"Throttle":.9},

	{"Aileron":-.3, "Rudder":.2, "Elevator":-.2, "Throttle":.9},	]

steps = [1, 2, 10, 100, 1000] #too many?  NAAAAH

tt.ttprint(tt.DETAIL, "running Update() iteratively for various numbers of steps with various initial conditions and applied controls")

for i, (steps, state, control) in enumerate(itertools.product(
		[1, 2, 10, 100, 1000], Update_initial_states, Update_controls)):
		Steps_procedure = lambda x: Update_procedure(x,steps)
		desc1 = "-".join(state.keys())
		desc2 = "-".join(control.keys())
		tm.test(f"Update_{i}_N{steps}_{desc1}_{desc2}", Steps_procedure,
			 {"initial_state":state, "controls":control})

tm.end_block()

#%% wrap it up by printing out a summary or by pickling our results:
tm.conclude()


