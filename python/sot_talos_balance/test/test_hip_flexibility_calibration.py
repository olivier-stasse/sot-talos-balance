from sot_talos_balance.utils.run_test_utils import *
from time import sleep

run_test('appli_hip_flexibility_calibration.py')

sleep(10.0)

raw_input("Wait before plugging the hip calibration")
runCommandClient('plug(robot.hipCalib.q_cmd, robot.device.control)')

sleep(10.0)
raw_input("Wait before leaving the simulation")
