from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph import plug

N_JOINTS = 32;

def create_joint_trajectory_generator(dt):
    jtg = NdTrajectoryGenerator("jtg");
    jtg.initial_value.value = tuple(32*[0.0]);
    jtg.trigger.value = 1.0;
    jtg.init(dt, N_JOINTS);
    return jtg;
