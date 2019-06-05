from sot_talos_balance.utils.run_test_utils import *
from sot_talos_balance.utils.plot_utils       import *

from time import sleep

run_test('appli_ankle_admittance_joint.py')

run_ft_calibration('robot.ftc')
raw_input("Wait before running the test")

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient('plug(robot.zmpEstimator.emergencyStop, robot.controlManager.emergencyStop_zmp)')
runCommandClient('plug(robot.dcmControl.zmpRef, robot.comAdmittanceController.zmpDes)')
runCommandClient('robot.comAdmittanceController.setState(robot.PG.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.comAdmittanceController.Kp.value = Kp_adm')
runCommandClient('robot.dcmControl.resetDcmIntegralError()')
runCommandClient('robot.dcmControl.Ki.value = [1.0,1.0,1.0]')


raw_input("Wait before dumping the data")

runCommandClient('dump_tracer(robot.tracer)')

