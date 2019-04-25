'''Test feet admittance control'''
from sot_talos_balance.utils.run_test_utils import *
from time import sleep

run_test('appli_feet_admittance.py')

run_ft_calibration('robot.ftc')
raw_input("Wait before running the test")

print('Set saturation value')
runCommandClient('robot.admBF_dqSaturation.sin.value = [0.0, 0.0, 0.01, 0.0, 0.0, 0.0]')

raw_input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

