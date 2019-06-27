from sot_talos_balance.utils.run_test_utils import *
from time import sleep

run_test('appli_ankle_joint_admittance.py')

run_ft_calibration('robot.ftc')
raw_input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmpEstimator.emergencyStop, robot.controlManager.emergencyStop_zmp)')
runCommandClient('robot.dcmControl.resetDcmIntegralError()')
runCommandClient('robot.dcmControl.Ki.value = [1.0,1.0,1.0]')
runCommandClient('robot.wrenchDistributor.wrenchDes.recompute(0)')

sleep(1.0)

RightPitchJoint = 10
LeftPitchJoint = 4
RightRollJoint = 11
LeftRollJoint = 5

raw_input("Wait before pushing the tasks")

runCommandClient('robot.sot.push(robot.rightAnklePitchTask.task.name)')
runCommandClient('robot.sot.push(robot.rightAnkleRollTask.task.name)')
runCommandClient('robot.sot.push(robot.leftAnklePitchTask.task.name)')
runCommandClient('robot.sot.push(robot.leftAnkleRollTask.task.name)')
runCommandClient('robot.sot.push(robot.taskCom.task.name)')
runCommandClient('robot.sot.push(robot.taskPosture.name)')

sleep(5.0)

tauRP = evalCommandClient('robot.device.ptorque.value[RightPitchJoint]')
tauLP = evalCommandClient('robot.device.ptorque.value[LeftPitchJoint]')
tauRR = evalCommandClient('robot.device.ptorque.value[RightRollJoint]')
tauLR = evalCommandClient('robot.device.ptorque.value[LeftRollJoint]')

tauDesRP = evalCommandClient('robot.rightPitchAnkleController.tauDes.value')
tauDesLP = evalCommandClient('robot.leftPitchAnkleController.tauDes.value')
tauDesRR = evalCommandClient('robot.rightRollAnkleController.tauDes.value')
tauDesLR = evalCommandClient('robot.leftRollAnkleController.tauDes.value')

print("Desired torques: %f, %f, %f, %f" % (tauDesRP[0], tauDesLP[0], tauDesRR[0], tauDesLR[0]))
print("Current torques: %f, %f, %f, %f" % (tauRP, tauLP, tauRR, tauLR))

c = ask_for_confirmation("Do you want to change the ankle admittance gains?")
if c:
    runCommandClient('robot.rightPitchAnkleController.Kp.value = [0.002]')
    runCommandClient('robot.leftPitchAnkleController.Kp.value = [0.002]')
    runCommandClient('robot.rightRollAnkleController.Kp.value = [0.002]')
    runCommandClient('robot.leftRollAnkleController.Kp.value = [0.002]')
        
    print("Gains changed to +/- 0.002")

tauRP = evalCommandClient('robot.device.ptorque.value[RightPitchJoint]')
tauLP = evalCommandClient('robot.device.ptorque.value[LeftPitchJoint]')
tauRR = evalCommandClient('robot.device.ptorque.value[RightRollJoint]')
tauLR = evalCommandClient('robot.device.ptorque.value[LeftRollJoint]')

tauDesRP = evalCommandClient('robot.rightPitchAnkleController.tauDes.value')
tauDesLP = evalCommandClient('robot.leftPitchAnkleController.tauDes.value')
tauDesRR = evalCommandClient('robot.rightRollAnkleController.tauDes.value')
tauDesLR = evalCommandClient('robot.leftRollAnkleController.tauDes.value')

print("Desired torques: %f, %f, %f, %f" % (tauDesRP[0], tauDesLP[0], tauDesRR[0], tauDesLR[0]))
print("Current torques: %f, %f, %f, %f" % (tauRP, tauLP, tauRR, tauLR))

c2 = ask_for_confirmation("Do you want to change the ankle admittance gains?")
if c2:
    runCommandClient('robot.rightPitchAnkleController.Kp.value = [0.005]')
    runCommandClient('robot.leftPitchAnkleController.Kp.value = [0.005]')
    runCommandClient('robot.rightRollAnkleController.Kp.value = [0.005]')
    runCommandClient('robot.leftRollAnkleController.Kp.value = [0.005]')
        
    print("Gains changed to +/- 0.005")