'''Test CoM admittance control as described in paper.'''
from sot_talos_balance.utils.run_test_utils import *
from time import sleep

import matplotlib.pyplot as plt
import numpy as np

run_test('appli_dcmZmpControl_estimator.py')

run_ft_calibration('robot.ftc')
raw_input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

c = ask_for_confirmation("Do you want to raise the foot?")
if c:
    runCommandClient('robot.comTrajGen.move(1,-0.08,10.0)')
    print("Putting the robot in position...")
    sleep(10.0)
    print("Robot is in position!")
    c2 = ask_for_confirmation("Confirm raising the foot?")
    if c2:
        print("Raising the foot...")
        runCommandClient('h = robot.dynamic.LF.value[2][3]')
        runCommandClient('robot.lfPosTrajGen.move(2,h+0.05,10.0)')
        sleep(10.0)
        print("Foot has been raised!")
    else:
        print("Not raising the foot")
else:
    print("Not raising the foot")

raw_input("Wait before leaving the simulation")

runCommandClient('dump_tracer(robot.tracer)')

