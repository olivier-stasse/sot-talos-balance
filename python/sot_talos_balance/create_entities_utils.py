from sot_talos_balance.joint_trajectory_generator import JointTrajectoryGenerator
from sot_talos_balance.joint_position_controller import JointPositionController
from dynamic_graph import plug

def create_joint_trajectory_generator(device, dt=0.001, robot_name="robot"):
    jtg = JointTrajectoryGenerator("jtg");
    plug(device.robotState,      jtg.base6d_encoders);
    jtg.init(dt, robot_name);
    return jtg;

def create_joint_position_controller(robot, dt=0.001, robot_name="robot"):
    jpc = JointPositionController("jpc");
    plug(robot.device.robotState,      jpc.q);
    return jpc;
