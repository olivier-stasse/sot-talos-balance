from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from sot_talos_balance.utils.gazebo_utils   import is_gazebo_present, GazeboLinkStatePublisher
from time                                   import sleep
import matplotlib.pyplot                    as plt
import numpy                                as np

pub_base = None
if is_gazebo_present():
    pub_base = GazeboLinkStatePublisher('base_link',1000,local_frame = False)
    print("Starting Gazebo link state publisher...")
    pub_base.start()
    print("Gazebo link state publisher started")

raw_input("Wait before running the test")

run_test('appli_dcm_estimator.py')

sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,3.0)')
sleep(10.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,1.0)')
sleep(5.0)

# # --- DUMP SIGNALS
# runCommandClient("dump_sot_sigs(robot,[[robot.base_estimator,'q','q_imu'],[robot.subscriber,'position','velocity']],5.)")
# sleep(6.)


# plt.ion()

raw_input("Wait before leaving the test")

if pub_base is not None:
    print("Stopping Gazebo link state publisher...")
    pub_base.stop()
    sleep(0.1)
    print("Gazebo link state publisher stopped")
