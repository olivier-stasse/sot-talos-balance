'''Test CoM admittance control as described in paper, with pre-loaded movements'''
from sot_talos_balance.utils.run_test_utils import *
from time import sleep

from sys import argv
test_folder = sys.argv[1] if len(argv)>1 else 'TestKajita2003WalkingOnSpot64/DSP20SSP780'
print('Using folder ' + test_folder)

runCommandClient('test_folder = "' + test_folder + '"')

run_test('appli_base_estimator.py')

raw_input("Wait before resetting madgwick")
runCommandClient('robot.imu_filters.setBeta(1e-3)')

c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerTrajGen.sin.value = 1')
else:
    print('Not executing the trajectory')

raw_input("Wait before dumping the data")


