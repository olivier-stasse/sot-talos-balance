from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep

from sot_talos_balance.utils.gazebo_utils import apply_force

import matplotlib.pyplot as plt
import numpy as np

run_test('appli_dcmZmpControl.py')

sleep(5.0)

# connect ZMP control signal and reset controllers
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.dynamic.com.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = [20.0,10.0,0.0]')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = [1.0,1.0,0.0]')

sleep(5.0)

print('Kick the robot...')
apply_force([-1000.0,0,0],0.01)
print('... kick!')

sleep(5.0)

runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
dcm_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.estimator.name')+ '-dcm.dat')
zmp_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-zmp.dat')
zmpDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dcm_control.name') + '-zmpRef.dat')
com_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')
comDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.com_admittance_control.name') + '-comRef.dat')

plt.ion()

plt.figure()
plt.plot(dcm_data[:,1],'b-')
plt.plot(dcm_data[:,2],'r-')
plt.title('DCM')
plt.legend(['x','y'])

plt.figure()
plt.plot(com_data[:,1],'b-')
plt.plot(comDes_data[:,1],'b--')
plt.plot(com_data[:,2],'r-')
plt.plot(comDes_data[:,2],'r--')
plt.plot(com_data[:,3],'g-')
plt.plot(comDes_data[:,3],'g--')
plt.title('COM real vs desired')
plt.legend(['Real x','Desired x','Real y','Desired y','Real z','Desired z'])

plt.figure()
plt.plot(zmp_data[:,1],'b-')
plt.plot(zmpDes_data[:,1],'b--')
plt.plot(zmp_data[:,2],'r-')
plt.plot(zmpDes_data[:,2],'r--')
plt.title('ZMP real vs desired')
plt.legend(['Real x','Desired x','Real y','Desired y'])

plt.figure()
plt.plot(zmp_data[:,1] - zmpDes_data[:,1],'b-')
plt.plot(zmp_data[:,2] - zmpDes_data[:,2],'r-')
plt.title('ZMP error')
plt.legend(['Error on x','Error on y'])

raw_input("Wait before leaving the simulation")

