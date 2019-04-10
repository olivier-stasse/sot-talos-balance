from sot_talos_balance.utils.run_test_utils import *
from time import sleep

import matplotlib.pyplot as plt
import numpy as np

run_test('appli_dcm_estimator.py')

run_ft_calibration('robot.ftc')
raw_input("Wait before running the test")

# plug ZMP emergency signal
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,1.0)')
sleep(20.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,2.0)')
sleep(20.0)

raw_input("Wait before leaving the simulation")

