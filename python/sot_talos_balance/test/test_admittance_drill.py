from time import sleep

from sot_talos_balance.utils.run_test_utils import run_ft_wrist_calibration, run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_admittance_drill.py')

run_ft_wrist_calibration('robot.forceCalibrator')

input("Wait before running the test")
input("Go to position for drill")
runCommandClient("go_to_position(robot.trajGen, robot.positionDrill, 5.0)")
runCommandClient("robot.comTrajGen.move(0,0.01,5.0)")
sleep(10.0)


input("Put the drill in the hand and continue to close the hand of the robot")
runCommandClient("robot.trajGen.move(35, -0.4, 5.0)")

sleep(8.0)

input("Wait before pushing the taskRightHand")

runCommandClient("robot.sot.remove(robot.taskPosture.name)")
sleep(5.0)
runCommandClient("robot.sot.push(robot.taskRightHand.task.name)")
sleep(2.0)
runCommandClient("robot.sot.push(robot.taskPosture.name)")

# robot.controller.w_forceDes.value = [.1, 0.0, 0.0, 0.0, 0.0, 0.0]
# robot.controller.dqSaturation.value = [.1, 0.0, 0.0, 0.0, 0.0, 0.0]
# robot.controller.Kp.value = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
# robot.controller.Kd.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

input("Wait before leaving the simulation")

runCommandClient('dump_tracer(robot.tracer)')
