from dynamic_graph.sot.core.operator import Mix_of_vector
from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from sot_talos_balance.joint_position_controller import JointPositionController
from sot_talos_balance.joint_admittance_controller import JointAdmittanceController
from sot_talos_balance.dummy_dcm_estimator import DummyDcmEstimator
from sot_talos_balance.com_admittance_controller import ComAdmittanceController

from time import sleep
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
    controller = JointAdmittanceController("admctrl")
    controller.Kp.value = Kp
    plug(robot.device.state,controller.state)

    mix = create_extend_mix(N_JOINTS,N_JOINTS+6)
    plug(robot.device.ptorque,mix.signal("sin2"))
    plug(mix.sout,controller.tau)

    # plug(robot.device.ptorque,controller.tau)

    controller.tauDes.value = [0.0]*(N_JOINTS+6)
    controller.init(dt, N_JOINTS+6)
    controller.setPosition(robot.device.state.value)
    return controller

def create_dummy_dcm_estimator(robot):
    from math import sqrt
    estimator = DummyDcmEstimator("dummy")
    mass = robot.dynamic.data.mass[0]
    robot.dynamic.com.recompute(0)
    h = robot.dynamic.com.value[2]
    g = 9.81
    omega = sqrt(g/h)

    estimator.mass.value = mass
    estimator.omega.value = omega
    plug(robot.dynamic.com,estimator.com)
    plug(robot.dynamic.momenta,estimator.momenta)
    estimator.init()
    return estimator

def create_com_admittance_controller(Kp,dt,robot):
    controller = ComAdmittanceController("comAdmCtrl")
    controller.Kp.value = Kp
    plug(robot.dynamic.zmp,controller.zmp)
    robot.dynamic.zmp.recompute(0)
    controller.zmpDes.value = robot.dynamic.zmp.value
    controller.ddcomDes.value = [0.0,0.0,0.0]

    controller.init(dt)
    robot.dynamic.com.recompute(0)
    controller.setState(robot.dynamic.com.value,[0.0,0.0,0.0])
    return controller

def addTrace(tracer, entity, signalName):
    signal = '{0}.{1}'.format(entity.name, signalName)
    filename = '{0}-{1}'.format(entity.name, signalName)
    tracer.add(signal, filename)

def dump_tracer(tracer):
    tracer.stop()
    sleep(0.2)
    tracer.dump()
    sleep(0.2)
    tracer.close()

