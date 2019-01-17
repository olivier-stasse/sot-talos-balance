from sot_talos_balance.test.run_test_utils import run_test, runCommandClient
from time import sleep

run_test('appli_COMTraj.py')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')

