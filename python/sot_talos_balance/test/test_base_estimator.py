'''Test CoM admittance control as described in paper, with pre-loaded movements'''
from sys import argv

from sot_talos_balance.utils.run_test_utils import ask_for_confirmation, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

test_folder = argv[1] if len(argv) > 1 else 'TestKajita2003WalkingOnSpot64/DSP20SSP780'
print('Using folder ' + test_folder)

runCommandClient('test_folder = "' + test_folder + '"')

run_test('appli_base_estimator.py')

c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerTrajGen.sin.value = 1')
else:
    print('Not executing the trajectory')

input("Wait before dumping the data")
