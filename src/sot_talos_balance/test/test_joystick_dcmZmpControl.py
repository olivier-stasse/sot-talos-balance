'''Test CoM admittance control as described in paper, with a Reference Velocity provided'''

## File adapted from appli_dcmZmpControl_file.py and
## appli_dcmZmpControl_online_ISA.py written by Isabelle Maroger
## ALL THINGS ADDED OR CHANGED ARE COMMENTED IN CAPITAL LETTERS

from sys import argv

from sot_talos_balance.utils.run_test_utils import *
from time import sleep

from dynamic_graph import * # for entity graph display

run_test('appli_joystick_dcmZmpControl.py')

run_ft_calibration('robot.ftc')
input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerPG.sin.value = 1')

else:
    print('Not executing the trajectory')

input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

print('Bye!')
