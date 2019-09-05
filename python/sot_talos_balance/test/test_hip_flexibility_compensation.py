from sot_talos_balance.utils.run_test_utils import *
from time import sleep

run_test('appli_hip_flexibility_compensation.py')

sleep(10.0)

raw_input("Wait before plugging the hip calibration")
runCommandClient('plug(robot.hipComp.q_cmd, robot.device.control)')
# runCommandClient('robot.hipComp.setLowPassFilterFrequency(1)')

sleep(10.0)
raw_input("Wait before leaving the simulation")
