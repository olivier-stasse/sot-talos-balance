'''Test CoM admittance control as described in paper, with pre-loaded movements'''
from sot_talos_balance.utils.run_test_utils import *
from time import sleep

run_test('appli_dcmZmpControl_file.py')

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

# Play the movements
c = ask_for_confirmation("Execute a sinusoid?")
if c:
    print("Executing the sinusoid...")
    runCommandClient('robot.comTrajGen.move(1,-0.025,1.0)')
    runCommandClient('robot.zmpTrajGen.move(1,-0.025,1.0)')
    sleep(5.0)
    runCommandClient('robot.comTrajGen.startSinusoid(1,0.025,2.0)')
    runCommandClient('robot.zmpTrajGen.startSinusoid(1,0.025,2.0)')

raw_input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

