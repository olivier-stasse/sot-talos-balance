from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from sot_talos_balance.utils.gazebo_utils import GazeboLinkStatePublisher
from time import sleep

pub = GazeboLinkStatePublisher('base_link',1000)
print("Starting Gazebo link state publisher...")
pub.start()
print("Gazebo link state publisher started")
raw_input("Wait before running the test")

run_test('appli_COMTraj_feedback_gazebo.py')

# wait for sensor values to be ready
sleep(evalCommandClient('robot.timeStep'))

# set initial conditions from sensor readings
runCommandClient('robot.rdynamic.com.recompute(0)')
runCommandClient('robot.comTrajGen.initial_value.value = robot.rdynamic.com.value')
runCommandClient('robot.contactLF.keep()')
runCommandClient('robot.contactRF.keep()')

# plug the SOT
runCommandClient('plug(robot.sot.control,robot.device.control)')

# execute rest of the commands
sleep(2.0)
runCommandClient('robot.comTrajGen.move(1,-0.025,4.0)')
sleep(5.0)
runCommandClient('robot.comTrajGen.startSinusoid(1,0.05,8.0)')

