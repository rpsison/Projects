"""At the moment, this test harness is incomplete, only used to demonstrate
that use_wind mode in VehicleAerodynamicsModel.py doesn't affect anything important"""

from TestTools import TestTools as tt

import math
import itertools

import sys
sys.path.append("..") #python is horrible, no?
from ece163.Modeling import VehicleGeometry as VG
from ece163.Utilities import Rotations

lab_name = "ECE163_Lab0"


parser = tt.parse_args_for_lab(lab_name)
args = parser.parse_args()
# args.generate = True #helpful for debugging, leave in for now

#make our testManager
tt.apply_args(args)
tm = tt.TestManager(lab_name, args)

#%%
tm.start_block("euler2DCM() tests")
def euler2DCM_procedure(inputs):
	
	#prepare inputs
	ypr={"yaw":0, "pitch":0, "roll":0}
	for key in inputs.keys():
		ypr[key] = inputs[key]
	
	#run student code
	R = Rotations.euler2DCM(ypr["yaw"],ypr["pitch"],ypr["roll"])
	return {"R":R}

yprs_to_test = [
	{},
	{"yaw":60},
	{"yaw":-60},
	{"yaw":90},
	{"yaw":-90},
	{"yaw":180},
	{"yaw":-180},
	{"pitch":60},
	{"pitch":-60},
	{"pitch":180},
	{"pitch":-180},
	{"roll":60},
	{"roll":-60},
	{"roll":90},
	{"roll":-90},
	{"roll":180},
	{"roll":-180},
	# a couple bagels with everythings:
	{"yaw":60, "pitch":60, "roll":60},
	{"yaw":-60, "pitch":-60, "roll":-60},
	# some limit breaks:
	{"yaw":60, "pitch":60, "roll":135},
	{"yaw":-60, "pitch":-60, "roll":-135},
	{"yaw":270, "pitch":60, "roll":60},
	{"yaw":-270, "pitch":-60, "roll":-60},
	{"yaw":60, "pitch":135, "roll":60},
	{"yaw":-60, "pitch":-135, "roll":-60},
	#some gimbal locks:
	{"pitch":90},
	{"pitch":-90},
	{"yaw":60, "pitch":90, "roll":60},
	{"yaw":-60, "pitch":-90, "roll":-60},
]	

for i, ypr in enumerate(yprs_to_test):
	desc = "-".join([f"{k}{v}" for k,v in ypr.items()])
	inpt = {k:tt.d2r(v) for k,v in ypr.items()}
	tm.test(f"euler2DCM_{i}_{desc}", euler2DCM_procedure,
		 inpt)
	
tm.end_block()


#%%
tm.start_block("dcm2Euler() tests")
def dcm2Euler_procedure(inputs):
	
	#setup dcm:
	dcm = inputs["dcm"]
	
	ret = {}
	ret["yaw"],ret["pitch"],ret["roll"] = Rotations.dcm2Euler(dcm)

	return ret

rs_to_test = [
	[[1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0]],

	[[0.8660254037844387, 0.49999999999999994, 0.0],
     [-0.49999999999999994, 0.8660254037844387, 0.0],
     [0.0, 0.0, 1.0]],


	[[0.8660254037844387, 0.0, -0.49999999999999994],
     [0.0, 1.0, 0.0],
     [0.49999999999999994, 0.0, 0.8660254037844387]],


	[[1.0, 0.0, 0.0],
     [0.0, 0.8660254037844387, 0.49999999999999994],
     [0.0, -0.49999999999999994, 0.8660254037844387]],


	[[0.7500000000000001, 0.4330127018922193, -0.49999999999999994],
	 [-0.21650635094610968, 0.8750000000000001, 0.4330127018922193],
	 [0.625, -0.21650635094610968, 0.7500000000000001]],

	
	[[0.8660254037844387, -0.49999999999999994, 0.0],
	 [0.49999999999999994, 0.8660254037844387, 0.0],
	 [0.0, 0.0, 1.0]],


	[[0.8660254037844387, 0.0, 0.49999999999999994],
	 [0.0, 1.0, 0.0],
	 [-0.49999999999999994, 0.0, 0.8660254037844387]],


	[[1.0, 0.0, 0.0],
	 [0.0, 0.8660254037844387, -0.49999999999999994],
	 [0.0, 0.49999999999999994, 0.8660254037844387]],


	[[0.75, -0.4330127018922193, 0.5],
     [0.6495190528383289, 0.6250000000000001, -0.4330127018922193],
     [-0.125, 0.6495190528383289, 0.75]],

	
	#A wraparound
	[[-0.7071067811865475, 0.7071067811865476, 0.0],
     [-0.7071067811865476, -0.7071067811865475, 0.0],
     [0.0, 0.0, 1.0]],

	
	[[-0.7071067811865475, 0.7071067811865476, 0.0],
     [-0.5000000000000001, -0.5, -0.7071067811865476],
     [-0.5000000000000001, -0.5, 0.7071067811865476]],

	#a few right angle ones:
	[[0, 0, 1],
     [0.0, 1.0, 0.0],
     [1, 0.0, 0]],

	[[0, 0, 1],
     [0.0, -1.0, 0.0],
     [-1, 0.0, 0]],

	[[0, 0, -1],
     [0.0, 1.0, 0.0],
     [-1, 0.0, 0]],

	
	#some gimbal locks:
	[[0, 1, 0],
     [0.0, 0, -1],
     [-1, 0.0, 0]],

	[[6.123233995736766e-17, 0.0, 1.0],
     [0.0, 1.0, 0.0],
     [-1.0, 0.0, 6.123233995736766e-17]],
	
 	[[6.123233995736766e-17, 0.0, -1.0],
     [0.0, 1.0, 0.0],
     [1.0, 0.0, 6.123233995736766e-17]],


	#a couple that break the limits:	
	[[0, 0, 1.001],
     [0.0, 1.0, 0.0],
     [1.001, 0.0, 0]],

	[[0, 0, -1.001],
     [0.0, 1.0, 0.0],
     [-1.001, 0.0, 0]],
]	


for i, r in enumerate(rs_to_test):
	inpt = {"dcm":r}
	tm.test(f"dcm2Euler_{i}", dcm2Euler_procedure,
		 inpt)
	
tm.end_block()

#%% 
tm.start_block("ned2enu() tests")

def ned2enu_procedure(inputs):
	
	points = inputs["ned_points"]
	result = Rotations.ned2enu(points)

	return {"enu_points":result}

ned_pts_to_test = [
	[[1,0,0]],
	[[0,1,0]],
	[[0,0,1]],
	
	[[1,2,3]],
	#one multipoint test:
	[[1,2,3],[4,5,6]],
	]

for i, pts in enumerate(ned_pts_to_test):
	inpt = {"ned_points":pts}
	tm.test(f"ned2enu_{i}", ned2enu_procedure,
		 inpt)
	
tm.end_block()

#%%
tm.start_block("getNewPoints() tests")

def getNewPoints_procedure(inputs):
	#setup:
	vertex = inputs["vertex"]
	pose={"x":0, "y":0, "z":0, "yaw":0, "pitch":0, "roll":0}
	for key in inputs["pose"].keys():
		pose[key] = inputs["pose"][key]
	vg = VG.VehicleGeometry()
	
	newPoints = vg.getNewPoints(**pose)

	return {"enu_points":newPoints[vertex]}

verts_to_test = [0, 6, 13]
poses_to_test = [
	{},
	{"x":5},
	{"y":5},
	{"z":5},
	{"x":5, "y":5, "z":5},
	{"yaw":tt.d2r(30)},
	{"pitch":tt.d2r(30)},
	{"roll":tt.d2r(30)},
	{"yaw":tt.d2r(30), "pitch":tt.d2r(30), "roll":tt.d2r(30)},
	{"yaw":tt.d2r(30), "pitch":tt.d2r(90), "roll":tt.d2r(30)},
	
	{"x":5, "y":5, "z":5, 
	  "yaw":tt.d2r(30), "pitch":tt.d2r(30), "roll":tt.d2r(30)},
	
	]

for i, (pose, vert) in enumerate(itertools.product(
		poses_to_test, verts_to_test)):
	inpt = {"vertex":vert, "pose":pose}
	desc = "-".join([k for k in pose.keys()])
	tm.test(f"getNewPoints_{i}_{desc}_{vert}", getNewPoints_procedure,
		 inpt)
	
tm.end_block()

# tm.end_block()



#%% wrap it up by printing out a summary or by pickling our results:
tm.conclude()



#%%Used to generate tests
# print(Rotations.euler2DCM(0,0,0))
# print(Rotations.euler2DCM(tt.d2r(30),0,0))
# print(Rotations.euler2DCM(0,tt.d2r(30),0))
# print(Rotations.euler2DCM(0,0,tt.d2r(30)))
# print(Rotations.euler2DCM(tt.d2r(30),tt.d2r(30),tt.d2r(30)))

# #%%
# print(Rotations.euler2DCM(tt.d2r(-30),0,0))
# print(Rotations.euler2DCM(0,tt.d2r(-30),0))
# print(Rotations.euler2DCM(0,0,tt.d2r(-30)))
# print(Rotations.euler2DCM(tt.d2r(-30),tt.d2r(-30),tt.d2r(-30)))
# #%%
# print(Rotations.euler2DCM(tt.d2r(90),tt.d2r(-0),tt.d2r(-90)))
# print(Rotations.euler2DCM(tt.d2r(45),tt.d2r(-90),tt.d2r(-45)))
# print(Rotations.euler2DCM(tt.d2r(0),tt.d2r(-90),tt.d2r(0)))
# print(Rotations.euler2DCM(tt.d2r(0),tt.d2r(90),tt.d2r(0)))
# #%%
# print(Rotations.euler2DCM(tt.d2r(135),tt.d2r(0),tt.d2r(0)))
#%%

