from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep

run_test('appli_admittance_end_effector.py')

sleep(1.0)

runCommandClient('robot.sot.push(taskRightHand.task.name)')
