from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from sot_talos_balance.joint_position_controller import JointPositionController

from dynamic_graph import plug

N_JOINTS = 32;

def create_joint_trajectory_generator(dt):
    jtg = NdTrajectoryGenerator("jtg");
    jtg.initial_value.value = tuple(N_JOINTS*[0.0]);
    jtg.trigger.value = 1.0;
    jtg.init(dt, N_JOINTS);
    return jtg;

def create_com_trajectory_generator(dt,robot):
    comTrajGen = NdTrajectoryGenerator("comTrajGen");
    comTrajGen.initial_value.value = robot.dynamic.com.value
    comTrajGen.trigger.value = 1.0;
    comTrajGen.init(dt, 3);
    return comTrajGen;

def create_joint_controller(Kp):
    controller = JointPositionController("posctrl")
    controller.Kp.value = Kp
    return controller
