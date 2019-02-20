from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep

import matplotlib.pyplot as plt
import numpy as np

run_test('appli_zmpEstimator.py')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(20.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')
sleep(20.0)

runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
zmpEst_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.zmp_estimator.name') + '-zmp.dat')
zmpDyn_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-zmp.dat')
com_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')

plt.ion()

plt.figure()
plt.plot(zmpEst_data[:,1],'b-')
plt.plot(zmpDyn_data[:,1],'b--')
plt.plot(com_data[:,1],'b:')
plt.plot(zmpEst_data[:,2],'r-')
plt.plot(zmpDyn_data[:,2],'r--')
plt.plot(com_data[:,2],'r:')
plt.title('ZMP estimate vs dynamic vs CoM (planar)')
plt.legend(['x estimate', 'x dynamic', 'x CoM', 'y estimate', 'y dynamic', 'y CoM'])

plt.figure()
plt.plot(com_data[:,1],'b-')
plt.plot(com_data[:,2],'r-')
plt.plot(com_data[:,3],'g-')
plt.title('COM')
plt.legend(['x','y','z'])

plt.figure()
plt.plot(zmpDyn_data[:,1],'b-')
plt.plot(zmpDyn_data[:,2],'r-')
plt.plot(zmpDyn_data[:,3],'g-')
plt.title('ZMP dynamic')
plt.legend(['x','y','z'])

plt.figure()
plt.plot(zmpEst_data[:,1],'b-')
plt.plot(zmpEst_data[:,2],'r-')
plt.plot(zmpEst_data[:,3],'g-')
plt.title('ZMP estimate')
plt.legend(['x','y','z'])

raw_input("Wait before leaving the simulation")

