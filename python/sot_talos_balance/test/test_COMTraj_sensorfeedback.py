from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep

run_test('appli_COMTraj_sensorfeedback.py')
print evalCommandClient('robot.dcm_estimator.c.value')
runCommandClient('robot.comTrajGen.initial_value.value = robot.dcm_estimator.c.value')
runCommandClient('plug(robot.comTrajGen.x, robot.taskCom.featureDes.errorIN)')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')

