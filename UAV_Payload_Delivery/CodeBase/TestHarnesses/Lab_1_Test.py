from TestTools import TestTools as tt

import math

import sys
sys.path.append("..") #python is horrible, no?
from ece163.Modeling import VehicleDynamicsModel as VDM
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Utilities import Rotations

lab_name = "ECE163_Lab1"

parser = tt.parse_args_for_lab(lab_name)
args = parser.parse_args()
# args.generate = True #helpful for debugging, leave in for now

#make our testManager
tm = tt.TestManager(lab_name, args)

	
#%% Unforced derivative tests	

tm.start_block("Freefall derivative tests")
#ok, so first you make a procedure...
def freefall_procedure(state_vars, keys_to_check):
# 	tt.ttprint(tt.DEBUG, f"running with inputs {state_vars}")
	
	#setup our testing state:
	testState = States.vehicleState()
	testVDM = VDM.VehicleDynamicsModel()
	testFm = Inputs.forcesMoments()
	
	#update the state with the values we care about:
	for key, value in state_vars.items():
		testState.__dict__[key] = value
	testState.R = Rotations.euler2DCM(testState.yaw, testState.pitch, testState.roll)
		
	#and now the student code does it's thing
	testVDM.setVehicleState(testState)
	dot = testVDM.derivative(testState, testFm)
	
	return {key:dot.__dict__[key] for key in keys_to_check}

#make more tests out of that procedure:
def freefall_P_dot(state_vars):
	return freefall_procedure(state_vars, ["pn","pe","pd"])
def freefall_uvw_dot(state_vars):
	return freefall_procedure(state_vars, ["u","v","w"])
def freefall_pqr_dot(state_vars):
	return freefall_procedure(state_vars, ["p","q","r"])
# def freefall_ypr_dot(state_vars): #they don't actually need this in Derivative so why add it?
# 	return freefall_procedure(state_vars, ["yaw","pitch","roll",])



freefall_test_params = [
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

#and then use testManager to do the procedure...
for fcn in [freefall_P_dot, 
			freefall_uvw_dot, 
			freefall_pqr_dot,]:
	tt.ttprint(tt.DETAIL, f"running {fcn.__name__} for various inputs")
	for i, params in enumerate(freefall_test_params):
		desc = "-".join(params.keys())
		tm.test(f"{fcn.__name__}_{i}_{desc}", fcn, params)

tm.end_block()

#%% We should do something similar for forcesMoments inputs

tm.start_block("Forced derivative tests")
def forced_procedure(state_vars, keys_to_check):
# 	tt.ttprint(tt.DEBUG, f"running with inputs {state_vars}")
	
	#setup our testing state:
	testState = States.vehicleState()
	testVDM = VDM.VehicleDynamicsModel()
	testFm = Inputs.forcesMoments()
	
	#update the state with the values we care about:
	for key, value in state_vars.items():
		if key in testState.__dict__.keys():
			testState.__dict__[key] = value
		else:
			testFm.__dict__[key] = value  #luckily forcesMoments and state don't share any variable names!
	testState.R = Rotations.euler2DCM(testState.yaw, testState.pitch, testState.roll)
 		
	#and now the student code does it's thing
	testVDM.setVehicleState(testState)
	dot = testVDM.derivative(testState, testFm)
	
	return {key:dot.__dict__[key] for key in keys_to_check}

#make more tests out of that procedure:
def forced_P_dot(state_vars):
	return forced_procedure(state_vars, ["pn","pe","pd"])
def forced_uvw_dot(state_vars):
	return forced_procedure(state_vars, ["u","v","w"])
def forced_pqr_dot(state_vars):
	return forced_procedure(state_vars, ["p","q","r"])
# def forced_ypr_dot(state_vars):
# 	return forced_procedure(state_vars, ["yaw","pitch","roll"])

forced_test_params = [
	{"Fx": 4},
	{"Fy": -3},
	{"Fz": -0.5},
	{"Mx": 4},
	{"My": -3},
	{"Mz": -0.5},
	#mix it up:
	{"Fx": 4, "Fy":-3	,"Mx": 4},
	{"Fz": -0.5, "My":3, "Mz": -0.5},
	#at least one gimbal lock:
	{"Fz": -0.5, "My":3, "Mz": -0.5, "yaw":tt.d2r(-75), "pitch":tt.d2r(-90)},
	#and at one where we just mess with everything
	{"Fx": 4, "Fy":-3	,"Mx": 4, "Fz": -0.5, "My":3, "Mz": -0.5, "pn":-4, "u":3, "q":2, "yaw":tt.d2r(30), "pitch":tt.d2r(-20), "roll":tt.d2r(-30) },
	]

#and then use testManager to do the procedure...
for fcn in [forced_P_dot, 
			forced_uvw_dot, 
			forced_pqr_dot]:
	tt.ttprint(tt.DETAIL, f"running {fcn.__name__} for various inputs")
	for i, params in enumerate(forced_test_params):
		desc = "-".join(params.keys())
		tm.test(f"{fcn.__name__}_{i}_{desc}", fcn, params)

tm.end_block()

#%% How about some Rexp() testing?

tm.start_block("Rexp tests")

def rexp_is_rot_procedure(inputs):
	testState = States.vehicleState()
	testDot =  States.vehicleState()
	testVDM = VDM.VehicleDynamicsModel()
	
	#update the state with the values we care about:
	for key, value in inputs["state_vars"].items():
		testState.__dict__[key] = value
	for key, value in inputs["dot_vars"].items():
		testDot.__dict__[key] = value
	testState.R = Rotations.euler2DCM(testState.yaw, testState.pitch, testState.roll)
	
	#and now the student code does it's thing
	R = testVDM.Rexp(VPC.dT, testState, testDot)
	
	ret_dict = {}
	ret_dict["row0_dot_row1"] = sum([R[0][i] * R[1][i] for i in range(3)])
	ret_dict["row1_dot_row2"] = sum([R[1][i] * R[2][i] for i in range(3)])
	ret_dict["row2_dot_row0"] = sum([R[2][i] * R[0][i] for i in range(3)])
	ret_dict["R_det"] = (
		 R[0][0] * (R[1][1]*R[2][2] - R[1][2]*R[2][1]) +
		-R[0][1] * (R[1][0]*R[2][2] - R[1][2]*R[2][0]) +
		 R[0][2] * (R[1][0]*R[2][1] - R[1][1]*R[2][0])
		)
	
	return ret_dict
	
def rexp_procedure(inputs):
# 	tt.ttprint(tt.DEBUG, f"running with inputs {state_vars}")
	
	#setup our testing state:
	testState = States.vehicleState()
	testDot =  States.vehicleState()
	testVDM = VDM.VehicleDynamicsModel()
	
	#update the state with the values we care about:
	for key, value in inputs["state_vars"].items():
		testState.__dict__[key] = value
	for key, value in inputs["dot_vars"].items():
		testDot.__dict__[key] = value
	testState.R = Rotations.euler2DCM(testState.yaw, testState.pitch, testState.roll)
		 
		 
	#and now the student code does it's thing
	R = testVDM.Rexp(VPC.dT, testState, testDot)
	
	return {f"R{i}{j}":R[i][j] for i in range(3) for j in range(3)}

R_test_params = [
	{"p": 4},
	{"q": -3},
	{"r": -0.5},
	{"p": 4, "q": -3, "r":-1},
	{"p": 4, "q": -3, "r":1},
	{"p": 4, "q":  3, "r":1},
	#some tiny angles:
	{"p": -.18},
	{"q": -.18},
	{"r": -.18},
	{"p": .009},
	{"q": .009},
	{"r": .009},
	{"p":  .005, "q": .005, "r": .005},
	{"p": -.005, "q": .005, "r":-.005},
	#classic one at gimbal lock:
	{"pitch":tt.d2r(90), "p":3, "q":2, "r":-1},
	{"pitch":tt.d2r(-90), "roll":tt.d2r(-90), "p":3, "q":2, "r":-1},
	]

tt.ttprint(tt.DETAIL, "running Rexp for various inputs with zero Pdot")
for i, params in enumerate(R_test_params):
	desc = "-".join(params.keys())
	tm.test(f"Rexp_nodot_orthonormality_{i}_{desc}",rexp_is_rot_procedure,
		   {"state_vars":params, "dot_vars":{}})
	tm.test(f"Rexp_nodot_{i}_{desc}",rexp_procedure,
		   {"state_vars":params, "dot_vars":{}})
tt.ttprint(tt.DETAIL, "running Rexp for various inputs with nonzero Pdot")
for i, params in enumerate(R_test_params):
	desc1 = "-".join(params.keys())
	dot_params = R_test_params[(i+1)%len(R_test_params)]
	desc2 = "-".join(dot_params.keys())
	tm.test(f"Rexp_orthonormality_{i}_{desc1}_{desc2}", rexp_is_rot_procedure,
		 {"state_vars":params, "dot_vars":dot_params})
	tm.test(f"Rexp_dot_{i}_{desc1}_{desc2}", rexp_procedure,
		 {"state_vars":params, "dot_vars":dot_params})
	

tm.end_block()

#%% ForwardEuler testing

tm.start_block("Forward Euler tests")
def FE_procedure(inputs):
# 	tt.ttprint(tt.DEBUG, f"running with inputs {state_vars}")
	
	#setup our testing state:
	testState = States.vehicleState()
	testDot =  States.vehicleState()
	testVDM = VDM.VehicleDynamicsModel()
	
	#update the state with the values we care about:
	for key, value in inputs["state_vars"].items():
		testState.__dict__[key] = value
	for key, value in inputs["dot_vars"].items():
		testDot.__dict__[key] = value
	testState.R = Rotations.euler2DCM(testState.yaw, testState.pitch, testState.roll)
		 
		 
	#and now the student code does it's thing
	fe = testVDM.ForwardEuler(VPC.dT, testState, testDot)
	
	keys_to_check = ["pn","pe","pd","u","v","w","p","q","r"]
	return {key:fe.__dict__[key] for key in keys_to_check}

#fe is very simple, so I don't know that we need lots and lots of tests here
FE_test_params = [
	{"u":3},
	{"u":2},
	{"pn":-4, "u":3, "q":2, "yaw":tt.d2r(30), "pitch":tt.d2r(-20), "roll":tt.d2r(-30) },
	{"pn":-4, "u":3, "q":2, "yaw":tt.d2r(30), "pitch":tt.d2r(-90), "roll":tt.d2r(-30) },
	##gimbal town, obviously:
	{"r":-4, "p":3, "q":2, "yaw":tt.d2r(30), "pitch":tt.d2r(-90), "roll":tt.d2r(-30) },	
	]

tt.ttprint(tt.DETAIL, "running Rexp for various inputs with nonzero Pdot")
for i, params in enumerate(FE_test_params):
	desc1 = "-".join(params.keys())
	dot_params = FE_test_params[(i+1)%len(FE_test_params)]
	desc2 = "-".join(dot_params.keys())
	tm.test(f"ForwardEuler_{i}_{desc1}_{desc2}", FE_procedure,
		 {"state_vars":params, "dot_vars":dot_params})
	

tm.end_block()

#%% ForwardEuler testing

tm.start_block("IntegrateState single-step tests")
def Integrate_onestep_procedure(inputs):
# 	tt.ttprint(tt.DEBUG, f"running with inputs {state_vars}")
	
	#setup our testing state:
	testState = States.vehicleState()
	testDot =  States.vehicleState()
	testVDM = VDM.VehicleDynamicsModel()
	
	#update the state with the values we care about:
	for key, value in inputs["state_vars"].items():
		testState.__dict__[key] = value
	for key, value in inputs["dot_vars"].items():
		testDot.__dict__[key] = value
	testState.R = Rotations.euler2DCM(testState.yaw, testState.pitch, testState.roll)
		 
		 
	#and now the student code does it's thing
	newState = testVDM.IntegrateState(VPC.dT, testState, testDot)
	
	keys_to_check = ["pn","pe","pd","u","v","w","p","q","r","yaw","pitch","roll","R","chi","Va","alpha","beta"]
	return {key:newState.__dict__[key] for key in keys_to_check}

#I think we can reuse freefall test params here
IntegrateState_params = freefall_test_params

tt.ttprint(tt.DETAIL, "running Rexp for various inputs with nonzero Pdot")
for i, params in enumerate(IntegrateState_params):
	desc1 = "-".join(params.keys())
	dot_params = IntegrateState_params[(i+1)%len(IntegrateState_params)]
	desc2 = "-".join(dot_params.keys())
	tm.test(f"IntegrateState_{i}_{desc1}_{desc2}", Integrate_onestep_procedure,
		 {"state_vars":params, "dot_vars":dot_params})
	

tm.end_block()

#%%
tm.start_block("Update tests")
def Update_procedure(inputs, iterations):
# 	tt.ttprint(tt.DEBUG, f"running with inputs {state_vars}")
	
	#setup our testing state:
	testState = States.vehicleState()
	testVDM = VDM.VehicleDynamicsModel()
	testFm = Inputs.forcesMoments()
	
	#update the state with the values we care about:
	for key, value in inputs["initial_state"].items():
			testState.__dict__[key] = value
	for key, value in inputs["forces"].items():
			testFm.__dict__[key] = value  #luckily forcesMoments and state don't share any variable names!
	testState.R = Rotations.euler2DCM(
		testState.yaw, testState.pitch, testState.roll)
 		
	#and now the student code does it's thing
	testVDM.setVehicleState(testState)
	for i in range(iterations):
		testVDM.Update(testFm)
	newState = testVDM.getVehicleState()
	
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

Update_fMs = [
	{},
	{"Fx":3, "Fy":-1, "Fz":2},
	{"Mx":2, "My":-1, "Mz":0.5},
	{"Fx":3, "Fy":-1, "Fz":2, "Mx":2, "My":-1, "Mz":0.5},
	]

steps = [1, 2, 10, 100, 1000] #too many?  NAAAAH

tt.ttprint(tt.DETAIL, "running Update() iteratively for various numbers of steps with various initial conditions and applied forces")

i=0
for steps in [1, 2, 10, 100, 1000]:
	Steps_procedure = lambda inputs: Update_procedure(inputs, steps)
	for state in Update_initial_states:
		desc1 = "-".join(state.keys())
		for fm in Update_fMs:
			desc2 = "-".join(fm.keys())
			tm.test(f"Update_{i}_{steps}-steps_{desc1}_{desc2}", Steps_procedure,
			 {"initial_state":state, "forces":fm})
			i+= 1

tm.end_block()
#%% wrap it up by printing out a summary or by pickling our results:
tm.conclude()


