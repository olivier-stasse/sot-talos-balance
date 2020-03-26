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


# input("Put the drill in the hand and continue to close the hand of the robot")
# runCommandClient("robot.trajGen.move(35, -0.4, 5.0)")
runCommandClient("robot.trajGen.move(34, -0.02, 5.0)")

input("Wait before switching to Admittance control")

runCommandClient("plug(robot.sot_adm.control, robot.controlManager.ctrl_sot_input)")

# In Gazebo add a wall at the position (0.67, -0.382, 1.125)
# Apply a force on the hand of the robot to activate the admittance control:
# rosservice call /gazebo/apply_body_wrench '{body_name: "arm_right_7_link", 
# reference_frame: "arm_right_7_link", wrench: { force: { x: 0, y: 0, z: -2 } }, start_time: 0, duration: -1 }'

# In a terminal run: rosrun dynamic_graph_bridge run_command
# And set the controller gains and desired force:
# robot.controller.w_forceDes.value = [.01, 0.0, 0.0, 0.0, 0.0, 0.0]
# robot.controller.Kp.value = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
# robot.controller.dqSaturation.value = [.001, 0.0, 0.0, 0.0, 0.0, 0.0]


input("Wait before leaving the simulation")

runCommandClient('dump_tracer(robot.tracer)')
