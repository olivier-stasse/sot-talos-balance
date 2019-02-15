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

run_test('appli_config_feedback_gazebo.py')

# wait for sensor values to be ready
raw_input("Wait before plugging the SOT")

# set initial conditions from sensor readings
runCommandClient('robot.statemix.sout.recompute(0)')
runCommandClient('robot.device.set(robot.statemix.sout.value)')
runCommandClient('robot.taskPos.featureDes.errorIN.value = robot.device.state.value')

# plug the SOT
runCommandClient('plug(robot.sot.control,robot.device.control)')

sleep(5.0)
runCommandClient('dump_tracer(robot.tracer)')

# --- DISPLAY
posDes_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.taskPos.featureDes.name') + '-errorIN.dat')
posSot_data = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.device.name') + '-state.dat')
pos_data    = np.loadtxt('/tmp/dg_' + evalCommandClient('robot.statemix.name') + '-sout.dat')

posErr_data = pos_data - posDes_data

plt.ion()

plt.figure()
plt.plot(pos_data[:,1],'b-')
plt.plot(posDes_data[:,1],'b--')
plt.plot(posSot_data[:,1],'b:')
plt.plot(pos_data[:,2],'r-')
plt.plot(posDes_data[:,2],'r--')
plt.plot(posSot_data[:,2],'r:')
plt.plot(pos_data[:,3],'g-')
plt.plot(posDes_data[:,3],'g--')
plt.plot(posSot_data[:,3],'g:')
plt.title('Pos real vs desired vs SOT')
plt.legend(['Real x','Desired x','SOT x','Real y','Desired y','SOT y','Real z','Desired z','SOT z'])

plt.figure()
plt.plot(posErr_data[:,1],'b-')
plt.plot(posErr_data[:,2],'r-')
plt.plot(posErr_data[:,3],'g-')
plt.title('Pos error')
plt.legend(['x','y','z'])

plt.figure()
plt.plot(pos_data[:,1],'b-')
plt.title('Pos real x')
plt.figure()
plt.plot(posDes_data[:,1],'b--')
plt.title('Pos desired x')
plt.figure()
plt.plot(posSot_data[:,1],'b:')
plt.title('Pos SOT x')

plt.figure()
plt.plot(pos_data[:,2],'r-')
plt.title('Pos real y')
plt.figure()
plt.plot(posDes_data[:,2],'r--')
plt.title('Pos desired y')
plt.figure()
plt.plot(posSot_data[:,2],'r:')
plt.title('Pos SOT y')

plt.figure()
plt.plot(pos_data[:,3],'g-')
plt.title('Pos real z')
plt.figure()
plt.plot(posDes_data[:,3],'g--')
plt.title('Pos desired z')
plt.figure()
plt.plot(posSot_data[:,3],'g:')
plt.title('Pos SOT z')

raw_input("Wait before leaving the simulation")
print("Stopping Gazebo link state publisher...")
pub.stop()
sleep(0.1)
print("Gazebo link state publisher stopped")


