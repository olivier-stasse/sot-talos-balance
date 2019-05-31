from __future__ import print_function

from sot_talos_balance.create_entities_utils import *
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
import sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
import numpy as np
import pinocchio as pin
from numpy.testing import assert_almost_equal as assertApprox

# --- General ---
print("--- General ---")

dt = 0.001
robot_name = 'robot'

halfSitting = [0.0, 0.0,  1.018213,  0.00  ,  0.0, 0.0, 1.0,                         #Free flyer
               0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708, #Left Leg
               0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708, #Right Leg
               0.0 ,  0.006761,                                                 #Chest
               0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1, -0.005,        #Left Arm
              -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,      #Right Arm
               0.,  0.                                                          #Head
]

q = np.matrix(halfSitting).T
print("q: %s\n" % str(q.flatten().tolist()[0]))

from rospkg import RosPack
rospack = RosPack()
urdfPath = rospack.get_path('talos_data')+"/urdf/talos_reduced.urdf"
urdfDir = [rospack.get_path('talos_data')+"/../"]

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
data = model.createData()
com = pin.centerOfMass(model,data,q)
pin.updateFramePlacements(model,data)
m = data.mass[0]
#com[1] = 0. # ensure perfect symmetry

print("com: %s\n" % str(com.flatten().tolist()[0]))

leftName = param_server_conf.footFrameNames['Left']
leftId = model.getFrameId(leftName)
leftPos  = data.oMf[leftId]
#leftPos.rotation = pin.utils.eye(3) # ensure perfect alignment
print( "%s: %d" % (leftName,leftId) )
print(leftPos)

rightName = param_server_conf.footFrameNames['Right']
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]
#rightPos.rotation = pin.utils.eye(3) # ensure perfect alignment
#pR = leftPos.translation # ensure perfect symmetry
#pR[1] = -pR[1]
#rightPos.translation = pR
print( "%s: %d" % (rightName,rightId) )
print(rightPos)

g = 9.81
fz = m*g
force      = [0.0, 0.0, fz]
lx = float(com[0])
tauy = -fz*lx
wrench      = force      + [  0.0, tauy,   0.0]

print( "total wrench: %s" % str(wrench) )

# --- Parameter server ---
print("--- Parameter server ---")

param_server = create_parameter_server(param_server_conf,dt)

# --- DistributeWrench ---
print("--- DistributeWrench ---")

distribute = DistributeWrench('distribute')

distribute.q.value = halfSitting
distribute.wrenchDes.value = wrench
distribute.rho.value = 0.5
distribute.phase.value = 0

distribute.set_right_foot_sizes(base_estimator_conf.RIGHT_FOOT_SIZES)
distribute.set_left_foot_sizes(base_estimator_conf.LEFT_FOOT_SIZES)
distribute.init(robot_name)

# --- Wrench distribution ---
print()
print("--- Wrench distribution ---")
distribute.phase.value = 0

print( "Expected wrenches in GLOBAL frame:" )

forceLeft  = [0.0, 0.0, fz/2]
forceRight = [0.0, 0.0, fz/2]
ly = float(leftPos.translation[1])
taux = fz*ly/2
wrenchLeft  = forceLeft  + [ taux, tauy/2, 0.0]
wrenchRight = forceRight + [-taux, tauy/2, 0.0]

print( "expected wrench: %s" % str(wrench) )
print( "expected left wrench: %s"  % str(wrenchLeft) )
print( "expected right wrench: %s" % str(wrenchRight) )

print( "CoP in LOCAL sole frame:" )

copLeft  = [float(com[0] - leftPos.translation[0]),  0., 0.]
copRight = [float(com[0] - rightPos.translation[0]), 0., 0.]

print( "expected left CoP: %s"  % str(copLeft) )
print( "expected right CoP: %s" % str(copRight) )
print()

distribute.zmpRef.recompute(0)

print( "resulting wrench: %s" % str(distribute.wrenchRef.value) )
assertApprox(wrench,distribute.wrenchRef.value,2)
print( "resulting left wrench: %s"  % str(distribute.wrenchLeft.value) )
assertApprox(wrenchLeft,distribute.wrenchLeft.value,3)
print( "resulting right wrench: %s" % str(distribute.wrenchRight.value) )
assertApprox(wrenchRight,distribute.wrenchRight.value,3)

distribute.copLeft.recompute(0)
distribute.copRight.recompute(0)

print( "resulting left CoP: %s"  % str(distribute.copLeft.value) )
assertApprox(copLeft,distribute.copLeft.value,3)
print( "resulting right CoP: %s" % str(distribute.copRight.value) )
assertApprox(copRight,distribute.copRight.value,3)

distribute.emergencyStop.recompute(0)
stop = distribute.emergencyStop.value
np.testing.assert_equal(stop,0)

# --- Wrench saturation (left) ---
print()
print("--- Wrench saturation ---")
print('NOTE: "predicted" wrench values are not accurate due to the foot saturation and as such they are not checked.')
print("CoP values are predicted under the assumption that they are at the foot border and as such they are checked.")

# --- Wrench saturation (left) ---
print()
print("--- Wrench saturation (left) ---")
distribute.phase.value = 1
distribute.phase.time = 1

print( "Expected wrenches in GLOBAL frame:" )

wrenchLeft  = wrench
wrenchRight = [0.0]*6

print( "expected wrench: %s" % str(wrench) )
print( "expected left wrench: %s"  % str(wrenchLeft) )

print( "CoP in LOCAL sole frame:" )

copLeft  = [float(com[0] - leftPos.translation[0]),  base_estimator_conf.RIGHT_FOOT_SIZES[3], 0.]
copRight = [0.]*3

print( "expected left CoP: %s"  % str(copLeft) )
print()

distribute.zmpRef.recompute(1)

print( "resulting wrench: %s" % str(distribute.wrenchRef.value) )
#assertApprox(wrench,distribute.wrenchRef.value,2)
print( "resulting left wrench: %s"  % str(distribute.wrenchLeft.value) )
#assertApprox(wrenchLeft,distribute.wrenchLeft.value,3)

distribute.copLeft.recompute(1)
distribute.copRight.recompute(1)

print( "resulting left CoP: %s"  % str(distribute.copLeft.value) )
assertApprox(copLeft,distribute.copLeft.value,3)

distribute.emergencyStop.recompute(0)
stop = distribute.emergencyStop.value
np.testing.assert_equal(stop,0)

# --- Wrench saturation (right) ---
print()
print("--- Wrench saturation (right) ---")
distribute.phase.value = -1
distribute.phase.time = 2

print( "Expected wrenches in GLOBAL frame:" )

wrenchLeft  = [0.0]*6
wrenchRight = wrench

print( "expected wrench: %s" % str(wrench) )
print( "expected right wrench: %s" % str(wrenchRight) )

print( "CoP in LOCAL sole frame:" )

copLeft  = [0.]*3
copRight = [float(com[0] - rightPos.translation[0]),  base_estimator_conf.RIGHT_FOOT_SIZES[2], 0.]

print( "expected right CoP: %s" % str(copRight) )
print()

distribute.zmpRef.recompute(2)

print( "resulting wrench: %s" % str(distribute.wrenchRef.value) )
#assertApprox(wrench,distribute.wrenchRef.value,2)
print( "resulting right wrench: %s" % str(distribute.wrenchRight.value) )
#assertApprox(wrenchRight,distribute.wrenchRight.value,3)

distribute.copLeft.recompute(2)
distribute.copRight.recompute(2)

print( "resulting right CoP: %s" % str(distribute.copRight.value) )
assertApprox(copRight,distribute.copRight.value,3)

distribute.emergencyStop.recompute(0)
stop = distribute.emergencyStop.value
np.testing.assert_equal(stop,0)
