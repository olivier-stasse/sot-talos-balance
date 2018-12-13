from dynamic_graph.sot.core.operator import Mix_of_vector
from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from sot_talos_balance.joint_position_controller import JointPositionController
from sot_talos_balance.admittance_controller_single_joint import AdmittanceControllerSingleJoint

from dynamic_graph import plug

N_JOINTS = 32;

def create_extend_mix(n_in,n_out):
    assert n_out>n_in
    mix_of_vector = Mix_of_vector( "mix " + str(n_in) + "-" + str(n_out) )

    mix_of_vector.setSignalNumber(3)

    n_diff = n_out-n_in
    mix_of_vector.addSelec(1,0,n_diff)
    mix_of_vector.addSelec(2,n_diff,n_in)

    mix_of_vector.default.value=[0.0]*n_out
    mix_of_vector.signal("sin1").value = [0.0]*n_diff
    mix_of_vector.signal("sin2").value = [2.0]*n_in

    return mix_of_vector

def create_joint_trajectory_generator(dt):
    jtg = NdTrajectoryGenerator("jtg");
    jtg.initial_value.value = tuple(N_JOINTS*[0.0]);
    jtg.trigger.value = 1.0;
    jtg.init(dt, N_JOINTS);
    return jtg;

def create_config_trajectory_generator(dt):
    N_CONFIG = N_JOINTS + 6
    jtg = NdTrajectoryGenerator("jtg");
    jtg.initial_value.value = tuple(N_CONFIG*[0.0]);
    jtg.trigger.value = 1.0;
    jtg.init(dt, N_CONFIG);
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

def create_admittance_controller(Kp,dt,robot):
    controller = AdmittanceControllerSingleJoint("admctrl")
    controller.Kp.value = Kp
    plug(robot.device.state,controller.state)

    mix = create_extend_mix(N_JOINTS,N_JOINTS+6)
    plug(robot.device.currents,mix.signal("sin2"))
    plug(mix.sout,controller.tau)

    # plug(robot.device.ptorque,controller.tau)

    controller.tauDes.value = [0.0]*(N_JOINTS+6)
    controller.init(dt, N_JOINTS+6)
    controller.setPosition(robot.device.state.value)
    return controller
