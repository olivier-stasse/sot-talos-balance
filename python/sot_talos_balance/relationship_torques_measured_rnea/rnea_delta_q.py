import pinocchio as pin
from numpy import genfromtxt
import numpy as np
from sot_talos_balance.utils.plot_utils import *
from scipy.signal import butter, lfilter, freqz
import math

pin.switchToNumpyArray()

control = genfromtxt('controls_54.txt', delimiter=' ')
torque = genfromtxt('torques_20_18_54.txt', delimiter=' ')
force = genfromtxt('forces_20_18_54.txt', delimiter=' ')

# Add null freeflyer
position = np.zeros((len(control), 39))
for i in range(len(control)):
    position[i] = np.concatenate(([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], control[i, 2:]))

acceleration = np.zeros((len(position), 38))
velocity = np.zeros((len(position), 38))
# torqueFilt = np.zeros((len(torque), len(torque[0,2:])))
time = np.linspace(0.0, len(velocity)*0.001, num=len(velocity))

# halfSitting = (
#      0., 0., 1.018213, 0., 0. , 0., 1.0,
#      0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
#      0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
#      0.0 ,  0.006761,
#      0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1, -0.005,
#      -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,
#      0.0,0.0)

# def butter_lowpass_filter(data, cutoff= 3.667, fs=80.0, order=5):
#     nyq = 0.5 * fs
#     normal_cutoff = cutoff / nyq
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     y = lfilter(b, a, data)
#     return y

# for j in range(len(torque[0,2:])):
#     torqueFilt[:,j] = butter_lowpass_filter(torque[:,j+2])


################# Create robot #########################################

# Set the path where the urdf file of the robot is registered
path = "/opt/openrobots/share"
urdf = path + '/talos_data/urdf/talos_reduced.urdf'
vector = pin.StdVec_StdString()
vector.extend(item for item in path)
# Create the robot wrapper from the urdf, it will give the model of the robot and its data
robot = pin.RobotWrapper.BuildFromURDF(urdf, vector, pin.JointModelFreeFlyer())
# srdf = path + '/talos_data/srdf/talos.srdf'

# Take the model of the robot and load its reference configurations
model = robot.model
# pin.loadReferenceConfigurations(model, srdf, False)
data = model.createData()

for i in range(len(acceleration)): 
    if (i > 6):
        velocity[i, 6:] = (position[i-1, 7:] - position[i, 7:])/0.001
        acceleration[i] = (velocity[i-1] - velocity[i])/0.001

###################### Calibration Force Sensors ####################

meanForces = np.zeros((4,6))
for f in range(1000, 2000):
    meanForces[0] += force[f,2:8]
    meanForces[1] += force[f,8:14]
    meanForces[2] += force[f,14:20]
    meanForces[3] += force[f,20:26]
meanForces = meanForces * 1e-3

for f in range(len(force)):
    force[f,2:8] -= meanForces[0] 
    force[f,8:14] -= meanForces[1]
    force[f,14:20] -= meanForces[2] 
    force[f,20:26] -= meanForces[3] 


###################### FreeFlyer ####################################

for k in range(len(position)):
    pin.forwardKinematics(model, data, position[k])
    oM1_qff = data.oMi[13].actInv(data.oMi[1])
    position[k, 0:3] = oM1_qff.translation
    q = pin.Quaternion(oM1_qff.rotation)
    position[k, 3] = q.x
    position[k, 4] = q.y
    position[k, 5] = q.z
    position[k, 6] = q.w
    log6 = pin.log6(oM1_qff)
    velocity[k, 0:6] = np.concatenate((log6.linear, log6.angular))
    acceleration[k, 0:6] = (velocity[k-1, 0:6] - velocity[k, 0:6])/0.001


###################### RNEA ####################################

tauWithForce = np.zeros((len(velocity), 38))
tauWithoutVelAccFF = np.zeros((len(velocity), 38))
tauWithoutForce = np.zeros((len(velocity), 38))
# q = np.zeros((len(position), 39))
# v = np.zeros((len(velocity), 38))
# a = np.zeros((len(acceleration), 38))
for i in range(len(velocity)):    
    q = position[i]
    v = velocity[i]
    a = acceleration[i]
    vectorForce = pin.StdVec_Force()
    for k in range(34): #taille model.joints -> freeflyer index en 0 (-1 pour vec q et v)
        if (k == model.getJointId("leg_left_6_joint")): # 5 (leg_left_6_link) + 1
            f = pin.Force(force[i,2:8])
        elif (k == model.getJointId("leg_right_6_joint")):
            f = pin.Force(force[i,14:20])
        elif (k == model.getJointId("arm_left_7_joint")):
            f = pin.Force(force[i,8:14])
        elif (k == model.getJointId("arm_right_7_joint")):
            f = pin.Force(force[i,20:26])
        else:
            f = pin.Force(np.array([0.,0.,0.,0.,0.,0.]))
        vectorForce.append(f)
    rForce = pin.rnea(model,data,q,v,a,vectorForce)
    r =  pin.rnea(model,data,q,v,a)
    v[0:6] = [0.,0.,0.,0.,0.,0.]
    a[0:6] = [0.,0.,0.,0.,0.,0.]
    rNoff = pin.rnea(model,data,q,v,a,vectorForce)
    tauWithForce[i] = np.squeeze(np.asarray(rForce))
    tauWithoutForce[i] = np.squeeze(np.asarray(r))
    tauWithoutVelAccFF[i] = np.squeeze(np.asarray(rNoff))


############## Root Mean square error ##############

def rmse(predictions, targets):
    return np.sqrt(((predictions - targets) ** 2).mean())

############## Find relation RNEA - MEASURES ##############
#
# xl = np.zeros((4,2)) # [a b] 
xl = np.array([[ 1.20588548,  -2.78033796 ],
               [ 0.94952097,  -12.96222896],
               [ 1.03490801,  -6.76122458 ],
               [ 0.98943679,   7.8162852  ]])
rmsel = np.zeros(4)
for i in range(4):
    A = np.ones((len(tauWithForce[20000:130000]), 2))
    A[:,0] = tauWithForce[20000:130000,i+6]
    b = torque[20000:130000,i+2]
    xl[i] = np.dot(np.linalg.pinv(A[20000:130000]), b[20000:130000]) #Ax = b 
    Ax = np.dot(A, xl[i])
    rmsel[i] = rmse(Ax, b)
    plt.figure(i)
    plt.plot(time[20000:130000],Ax, 'r', label='left_leg joint ' + str(i) + ' reconstructed rnea')
    plt.plot(time[20000:130000],b, 'b', label='left_leg joint ' + str(i) + ' measured')
    plt.plot(time[20000:130000],tauWithForce[20000:130000,i+6], 'g', label='left_leg joint ' + str(i) + ' rnea')
    plt.title("Torque comparison left leg")
    plt.legend()
print(xl)
print(rmsel)
# rmsel = [ 0.60252838  1.98547641  0.66161414  1.05223428] : no calib static -> 48
# rmsel = [ 0.57108472  2.06281844  0.63074937  1.03549187] : Calib + Static -> 48
# rmsel = [ 0.71895333  2.43895242  1.53974824  1.30717949] : Calib + Push -> 54

# Left
# No calib static -> 48
# [[  0.97879666  -2.10031143]-> hip yaw
#  [  0.96595697 -11.51724871]-> hip roll
#  [  1.05060233  -3.97303081]-> hip pitch
#  [  0.99015236   8.93334293]] -> knee
# Calib + Static -> 48
# [[ 1.20588548   -2.78033796 ]
#  [ 0.94952097   -12.96222896]
#  [ 1.03490801   -6.76122458 ]
#  [ 0.98943679    7.8162852  ]]


# xr = np.zeros((4,2))
xr = np.array([[0.44766947,  -0.5805577 ],
               [0.92794379,  -6.54577756],
               [0.24191495,  -5.34795378],
               [1.21176435, -11.96719837]])
rmser = np.zeros(4)
for i in range(6, 10):
    A = np.ones((len(tauWithForce[20000:130000]), 2))
    A[:,0] = tauWithForce[20000:130000,i+6]
    b = torque[20000:130000,i+2]
    xr[i-6] = np.dot(np.linalg.pinv(A[20000:130000]), b[20000:130000]) #Ax = b 
    Ax = np.dot(A, xr[i-6])
    rmser[i-6] = rmse(Ax, b)
    plt.figure(i)
    plt.plot(time[20000:130000],Ax, 'r', label='right_leg joint ' + str(i) + ' reconstructed rnea')
    plt.plot(time[20000:130000],b, 'b', label='right_leg joint ' + str(i) + ' measured')
    plt.plot(time[20000:130000],tauWithForce[20000:130000,i+6], 'g', label='right_leg joint ' + str(i) + ' rnea')
    plt.title("Torque comparison right leg")
    plt.legend()
print(xr)
print(rmser)
# rmser = [ 0.46053209  3.5433265   3.61696477  5.26060387] : no calib static -> 48
# rmser = [ 0.42935486  3.25796876  3.61012659  5.24385532] : Calib + Static -> 48
# rmser = [ 1.03004169  1.61199206  7.36712387  4.45158222] : Calib + Push -> 54

# Right
# No calib static -> 48
# [[  0.3349505   -0.55781495]-> hip yaw
#  [  0.97043399  -4.47099726]-> hip roll
#  [  0.3253965   -8.92303056]-> hip pitch
#  [  1.14857289 -23.51207438]] -> knee
# Calib + Static -> 48
# [[  0.44766947  -0.5805577 ]
#  [  0.92794379  -6.54577756]
#  [  0.24191495  -5.34795378]
#  [  1.21176435 -11.96719837]]
#  
#  
######################## forwardDynamics ##################################

# tauHipRollLeft = tauWithoutForce[:,1+6]
# tauHipRollRight = tauWithoutForce[:,7+6]

# deltaQHipRollLeft = tauHipRollLeft / 973.201390037
# deltaQHipRollRight = tauHipRollRight / 973.201390037

# #positionMod = np.zeros((len(position), len(position[0,2:])))
# positionMod = np.copy(q)
# positionMod[:, 1] += deltaQHipRollLeft
# positionMod[:, 7] += deltaQHipRollRight
# J = np.zeros(len(v[0,:]))
# J[17] = 1 # warning ff
# pin.forwardDynamics(model, data, positionMod[0], v[0], tauWithoutForce[0], J, np.array([0.0]))


######################## PLOTS ##################################


# plt.figure(1)
# plt.plot(time,tauWithForce[:,0+6], 'r', label='left_leg hip yaw rnea')
# plt.plot(time,tauWithoutVelAccFF[:,0+6], 'g', label='left_leg hip yaw rnea no vel acc')
# plt.plot(time,torque[:,0+2], 'b', label='left_leg hip yaw measured')
# plt.title("Torque comparison left leg hip yaw")
# plt.legend()

# plt.figure(2)
# plt.plot(time,tauWithForce[:,1+6], 'r', label='left_leg hip roll')
# plt.plot(time,tauWithoutVelAccFF[:,1+6], 'g', label='left_leg hip roll rnea no vel acc')
# plt.plot(time,torque[:,1+2], 'b', label='left_leg hip roll')
# plt.title("Torque comparison left leg hip roll")
# plt.legend()
# plt.figure(3)
# plt.plot(time,torque[:,2+2], 'b', label='left_leg hip pitch')
# plt.plot(time,tauWithoutVelAccFF[:,2+6], 'g', label='left_leg hip pitch rnea no vel acc')
# plt.plot(time,tauWithForce[:,2+6], 'r', label='left_leg hip pitch')
# plt.title("Torque comparison left leg hip pitch")
# plt.legend()
# plt.figure(4)
# plt.plot(time,torque[:,3+2], 'b', label='left_leg knee')
# plt.plot(time,tauWithoutVelAccFF[:,3+6], 'g', label='left_leg knee rnea no vel acc')
# plt.plot(time,tauWithForce[:,3+6], 'r', label='left_leg knee')
# plt.title("Torque comparison left leg knee")
# plt.legend()
plt.figure(15)
plt.plot(time,force[:,0+2], 'r', label='left_foot force x')
plt.plot(time,force[:,1+2], 'b', label='left_foot force y')
plt.plot(time,force[:,2+2], 'g', label='left_foot force z')
plt.title("Forces left foot")
plt.legend()

plt.figure(16)
plt.plot(time,force[:,12+2], 'r', label='right_foot force x')
plt.plot(time,force[:,13+2], 'b', label='right_foot force y')
plt.plot(time,force[:,14+2], 'g', label='right_foot force z')
plt.title("Forces Right foot")
plt.legend()

# plt.figure(2)
# plt.plot(time,tauWithForce[:,0+6], 'r', label='left_leg hip yaw')
# plt.plot(time,tauWithForce[:,1+6], 'b', label='left_leg hip roll')
# plt.plot(time,tauWithForce[:,2+6], 'g', label='left_leg hip pitch')
# plt.plot(time,tauWithForce[:,3+6], 'c', label='left_leg knee')
# plt.title("Torque with force left leg")
# plt.legend()

# diff = tauWithForce - tauWithoutForce

# plt.figure(3)
# plt.plot(time,diff[:,0+6], 'r', label='left_leg hip yaw')
# plt.plot(time,diff[:,1+6], 'b', label='left_leg hip roll')
# plt.plot(time,diff[:,2+6], 'g', label='left_leg hip pitch')
# plt.plot(time,diff[:,3+6], 'c', label='left_leg knee')
# plt.title("diff rnea left leg")
# plt.legend()

# plt.figure(7)
# plt.plot(time,torque[:,0+2], 'r', label='left_leg hip yaw')
# plt.plot(time,torque[:,1+2], 'b', label='left_leg hip roll')
# plt.plot(time,torque[:,2+2], 'g', label='left_leg hip pitch')
# plt.plot(time,torque[:,3+2], 'c', label='left_leg knee')
# plt.title("Torque measured left leg")
# plt.legend()

plt.show()