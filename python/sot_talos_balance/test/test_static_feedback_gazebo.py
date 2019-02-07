from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from sot_talos_balance.utils.gazebo_utils import GazeboLinkStatePublisher
from time import sleep

import matplotlib.pyplot as plt
import numpy as np

pub = GazeboLinkStatePublisher('base_link',5000)
print("Starting Gazebo link state publisher...")
pub.start()
print("Gazebo link state publisher started")
raw_input("Wait before running the test")

run_test('appli_static_feedback_gazebo.py')

# wait for sensor values to be ready
raw_input("Wait before plugging the SOT")

# set initial conditions from sensor readings
runCommandClient('robot.rdynamic.com.recompute(0)')
runCommandClient('robot.taskCom.featureDes.errorIN.value = robot.rdynamic.com.value')
runCommandClient('robot.device.set(robot.subscriber.position.value+robot.device.state.value[6:])')
runCommandClient('robot.contactLF.keep()')
runCommandClient('robot.contactRF.keep()')

# plug the SOT
runCommandClient('plug(robot.sot.control,robot.device.control)')

sleep(5.0)
runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
comDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.taskCom.featureDes.name') + '-errorIN.dat')
comSot_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.dynamic.name') + '-com.dat')
com_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.rdynamic.name') + '-com.dat')

comErr_data = com_data - comDes_data

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
plt.plot(comErr_data[:,1],'b-')
plt.plot(comErr_data[:,2],'r-')
plt.plot(comErr_data[:,3],'g-')
plt.title('COM error')
plt.legend(['x','y','z'])

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
print("Stopping Gazebo link state publisher...")
pub.stop()
sleep(0.1)
print("Gazebo link state publisher stopped")


