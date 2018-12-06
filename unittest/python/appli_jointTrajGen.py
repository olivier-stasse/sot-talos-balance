from sot_talos_balance.joint_trajectory_generator import JointTrajectoryGenerator
from dynamic_graph import plug

dt = robot.timeStep;

robot.traj_gen = JointTrajectoryGenerator("jtg");
plug(robot.device.robotState, robot.traj_gen.base6d_encoders);
robot.traj_gen.init(dt, "robot");
plug(robot.traj_gen.dq, robot.device.control);



robot.device.control.recompute(0)
