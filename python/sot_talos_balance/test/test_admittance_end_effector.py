from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep
from sot_talos_balance.utils.gazebo_utils import apply_force

run_test('appli_admittance_end_effector.py')

sleep(1.0)

runCommandClient('robot.sot.push(taskRightHand.task.name)')

sleep(50.0)

forceWorldFrame = evalCommandClient('robot.admittanceController.forceWorldFrame.value')

print("Current force: ")
print(forceWorldFrame)

runCommandClient('robot.admittanceController.forceDes.value = robot.admittanceController.forceWorldFrame.value')

# runCommandClient('robot.admittanceController.Kp.value = (0.00001, 0.00001, 0.00001, 0.0, 0.0, 0.0)')
