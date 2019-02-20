from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from sot_talos_balance.utils.gazebo_utils   import GazeboLinkStatePublisher
from time                                   import sleep
from sot_talos_balance.utils.plot_utils     import * 
import matplotlib.pyplot                    as plt
import numpy                                as np
from IPython import embed

# pub_base = GazeboLinkStatePublisher('base_link',1000)
pub_torso = GazeboLinkStatePublisher('torso_2_link',1000,local_frame = False)

print("Starting Gazebo link state publisher...")
# pub_base.start()
pub_torso.start()
print("Gazebo link state publisher started")
raw_input("Wait before running the test")

run_test('appli_dcm_estimator.py')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')
sleep(5.0)
runCommandClient("write_pdf_graph('/tmp/')")
	
# # --- DUMP SIGNALS
# runCommandClient("dump_sot_sigs(robot,[[robot.base_estimator,'q','q_imu'],[robot.subscriber,'position','velocity']],5.)")
# sleep(6.)


# plt.ion()

raw_input("Wait before leaving the simulation")
print("Stopping Gazebo link state publisher...")
pub_torso.stop()
sleep(0.1)
print("Gazebo link state publisher stopped")
write_pdf_graph('/tmp/')
