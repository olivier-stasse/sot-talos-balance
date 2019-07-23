'''Test CoM admittance control as described in paper, with pre-loaded movements'''
from sot_talos_balance.utils.run_test_utils import *
from time import sleep

from sys import argv
test_folder = sys.argv[1] if len(argv)>1 else 'TestKajita2003WalkingOnSpot64/DSP20SSP780'
print('Using folder ' + test_folder)

runCommandClient('test_folder = "' + test_folder + '"')

run_test('appli_dcmZmpControl_file.py')

raw_input("Wait before resetting madgwick")
runCommandClient('robot.imu_filters.setBeta(1e-3)')
c = ask_for_confirmation('Reset foot positions?')
if c:
    print('Resetting foot positions')
    runCommandClient('robot.base_estimator.reset_foot_positions()')
else:
    print('Not resetting foot positions')

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

c = ask_for_confirmation('Execute trajectory?')
if c:
    print('Executing the trajectory')
    runCommandClient('robot.triggerTrajGen.sin.value = 1')
else:
    print('Not executing the trajectory')

raw_input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

