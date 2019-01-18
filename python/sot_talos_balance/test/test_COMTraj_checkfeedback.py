from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from sot_talos_balance.test.link_state_publisher import LinkStatePublisher
from time import sleep

import matplotlib.pyplot as plt
import numpy as np

pub = LinkStatePublisher('base_link',1000)
print("Starting link state publisher...")
pub.start()
print("Link state publisher started")
raw_input("Wait before running the test")

run_test('appli_COMTraj_checkfeedback.py')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')
sleep(5.0)
runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
comDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.comTrajGen.name') + '-x.dat')
comSot_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')
com_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.rdynamic.name') + '-com.dat')

plt.ion()

plt.figure()
plt.plot(com_data[:,1],'b-')
plt.plot(comDes_data[:,1],'b--')
plt.plot(comSot_data[:,1],'b:')
plt.plot(com_data[:,2],'r-')
plt.plot(comDes_data[:,2],'r--')
plt.plot(comSot_data[:,2],'r:')
plt.plot(com_data[:,3],'g-')
plt.plot(comDes_data[:,3],'g--')
plt.plot(comSot_data[:,3],'g:')
plt.title('COM real vs desired vs SOT')
plt.legend(['Real x','Desired x','SOT x','Real y','Desired y','SOT y','Real z','Desired z','SOT z'])

plt.figure()
plt.plot(com_data[:,1],'b-')
plt.title('COM real x')
plt.figure()
plt.plot(comDes_data[:,1],'b--')
plt.title('COM desired x')
plt.figure()
plt.plot(comSot_data[:,1],'b:')
plt.title('COM SOT x')

plt.figure()
plt.plot(com_data[:,2],'r-')
plt.title('COM real y')
plt.figure()
plt.plot(comDes_data[:,2],'r--')
plt.title('COM desired y')
plt.figure()
plt.plot(comSot_data[:,2],'r:')
plt.title('COM SOT y')

plt.figure()
plt.plot(com_data[:,3],'g-')
plt.title('COM real z')
plt.figure()
plt.plot(comDes_data[:,3],'g--')
plt.title('COM desired z')
plt.figure()
plt.plot(comSot_data[:,3],'g:')
plt.title('COM SOT z')

raw_input("Wait before leaving the simulation")
print("Stopping link state publisher...")
pub.stop()
sleep(0.1)
print("Link state publisher stopped")

