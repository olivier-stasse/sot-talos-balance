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
print("q:")
print(q.flatten().tolist()[0])

from rospkg import RosPack
rospack = RosPack()
urdfPath = rospack.get_path('talos_data')+"/urdf/talos_reduced.urdf"
urdfDir = [rospack.get_path('talos_data')+"/../"]

model = pin.buildModelFromUrdf(urdfPath, pin.JointModelFreeFlyer())
data = model.createData()
com = pin.centerOfMass(model,data,q)
pin.updateFramePlacements(model,data)
m = data.mass[0]

#print("com:")
#print(com.flatten().tolist()[0])

leftName = param_server_conf.footFrameNames['Left']
leftId = model.getFrameId(leftName)
leftPos  = data.oMf[leftId]
#print( "%s: %d" % (leftName,leftId) )
#print(leftPos)

rightName = param_server_conf.footFrameNames['Right']
rightId = model.getFrameId(rightName)
rightPos = data.oMf[rightId]
#print( "%s: %d" % (rightName,rightId) )
#print(rightPos)

print( "wrenches in GLOBAL frame:" )

g = 9.81
fz = m*g
force      = [0.0, 0.0, fz]
forceLeft  = [0.0, 0.0, fz/2]
forceRight = [0.0, 0.0, fz/2]
lever = float(com[0])
tauy = -fz*lever
wrench      = force      + [0.0, tauy,   0.0]
wrenchLeft  = forceLeft  + [0.0, tauy/2, 0.0]
wrenchRight = forceRight + [0.0, tauy/2, 0.0]

print( "desired wrench: %s" % str(wrench) )
print( "expected left wrench: %s"  % str(wrenchLeft) )
print( "expected right wrench: %s" % str(wrenchRight) )

print( "CoP in LOCAL sole frame:" )

copLeft  = [float(com[0] - leftPos.translation[0]),  -float(leftPos.translation[1]), 0.]
copRight = [float(com[0] - rightPos.translation[0]), -float(rightPos.translation[1]), 0.]

print( "expected left CoP: %s"  % str(copLeft) )
print( "expected right CoP: %s" % str(copRight) )

# --- Parameter server ---
print("--- Parameter server ---")

param_server = create_parameter_server(param_server_conf,dt)

# --- Wrench distribution ---
print("--- Wrench distribution ---")

distribute = DistributeWrench('distribute')

distribute.q.value = halfSitting
distribute.wrenchDes.value = wrench

distribute.init(robot_name)

distribute.zmpRef.recompute(0)

print( "resulting wrench: %s" % str(distribute.wrenchRef.value) )
assertApprox(wrench,distribute.wrenchRef.value,6)
print( "resulting left wrench: %s"  % str(distribute.wrenchLeft.value) )
assertApprox(wrenchLeft,distribute.wrenchLeft.value,6)
print( "resulting right wrench: %s" % str(distribute.wrenchRight.value) )
assertApprox(wrenchRight,distribute.wrenchRight.value,6)

distribute.copLeft.recompute(0)
distribute.copRight.recompute(0)

print( "resulting left CoP: %s"  % str(distribute.copLeft.value) )
assertApprox(copLeft,distribute.copLeft.value,3)
print( "resulting right CoP: %s" % str(distribute.copRight.value) )
assertApprox(copRight,distribute.copRight.value,3)

distribute.emergencyStop.recompute(0)
stop = distribute.emergencyStop.value
np.testing.assert_equal(stop,0)

