from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep

run_test('appli_COMTraj_sensorfeedback.py')

# wait for sensor values to be ready
sleep(evalCommandClient('robot.timeStep'))

# set initial conditions from sensor readings
runCommandClient('robot.dcm_estimator.c.recompute(0)')
runCommandClient('robot.comTrajGen.initial_value.value = robot.dcm_estimator.c.value')
runCommandClient('robot.contactLF.keep()')
runCommandClient('robot.contactRF.keep()')

# plug the SOT
runCommandClient('plug(robot.sot.control,robot.device.control)')

# execute rest of the commands
sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')

