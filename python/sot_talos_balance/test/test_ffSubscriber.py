from sot_talos_balance.create_entities_utils import create_com_trajectory_generator
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from numpy import eye
from time import sleep
import os

from dynamic_graph.ros import RosSubscribe

from dynamic_graph.tracer_real_time import TracerRealTime
from sot_talos_balance.create_entities_utils import addTrace, dump_tracer
import matplotlib.pyplot as plt
import numpy as np

def main(robot):
    dt = robot.timeStep;
    robot.comTrajGen = create_com_trajectory_generator(dt,robot);

    # --- COM
    robot.taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    robot.taskCom.task.controlGain.value = 10

    # --- CONTACTS
    #define contactLF and contactRF
    robot.contactLF = MetaTaskKine6d('contactLF',robot.dynamic,'LF',robot.OperationalPointsMap['left-ankle'])
    robot.contactLF.feature.frame('desired')
    robot.contactLF.gain.setConstant(100)
    robot.contactLF.keep()
    locals()['contactLF'] = robot.contactLF

    robot.contactRF = MetaTaskKine6d('contactRF',robot.dynamic,'RF',robot.OperationalPointsMap['right-ankle'])
    robot.contactRF.feature.frame('desired')
    robot.contactRF.gain.setConstant(100)
    robot.contactRF.keep()
    locals()['contactRF'] = robot.contactRF

    robot.subscriber = RosSubscribe("torso_subscriber")
    robot.subscriber.add("vector","position","/sot/torso_1_link/position")
    robot.subscriber.add("vector","velocity","/sot/torso_1_link/velocity")

    robot.sot = SOT('sot')
    robot.sot.setSize(robot.dynamic.getDimension())
    plug(robot.sot.control,robot.device.control)

    robot.sot.push(robot.contactRF.task.name)
    robot.sot.push(robot.taskCom.task.name)
    robot.sot.push(robot.contactLF.task.name)
    robot.device.control.recompute(0)

    # --- TRACER
    robot.tracer = TracerRealTime("zmp_tracer")
    robot.tracer.setBufferSize(80*(2**20))
    robot.tracer.open('/tmp','dg_','.dat')
    robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))
    robot.device.after.addSignal('{0}.position'.format(robot.subscriber.name)) # force recalculation
    robot.device.after.addSignal('{0}.velocity'.format(robot.subscriber.name)) # force recalculation

    addTrace(robot.tracer, robot.dynamic, 'com')
    addTrace(robot.tracer, robot.subscriber, 'position')
    addTrace(robot.tracer, robot.subscriber, 'velocity')

    plug(robot.comTrajGen.x,    robot.taskCom.featureDes.errorIN);

    robot.tracer.start()

    sleep(1.0);
    os.system("rosservice call \start_dynamic_graph")
    sleep(2.0);
    robot.comTrajGen.move(1,-0.025,4.0);
    sleep(20.0);
    robot.comTrajGen.startSinusoid(1,0.05,8.0);
    sleep(5.0);

    dump_tracer(robot.tracer)

	  # --- DISPLAY
    com_data = np.loadtxt('/tmp/dg_'+robot.dynamic.name+'-com.dat')
    pos_data = np.loadtxt('/tmp/dg_'+robot.subscriber.name+'-position.dat')
    vel_data = np.loadtxt('/tmp/dg_'+robot.subscriber.name+'-velocity.dat')

    plt.figure()
    plt.plot(com_data[:,1],'b-')
    plt.plot(com_data[:,2],'r-')
    plt.plot(com_data[:,3],'g-')
    plt.title('COM')
    plt.legend(['x','y','z'])

    plt.figure()
    plt.plot(pos_data[:,1],'b-')
    plt.plot(pos_data[:,2],'r-')
    plt.plot(pos_data[:,3],'g-')
    plt.title('Position measure')
    plt.legend(['x','y','z'])

    plt.figure()
    plt.plot(pos_data[:,4],'b-')
    plt.plot(pos_data[:,5],'r-')
    plt.plot(pos_data[:,6],'g-')
    plt.title('Orientation measure')
    plt.legend(['yaw','pitch','roll'])

    plt.figure()
    plt.plot(vel_data[:,1],'b-')
    plt.plot(vel_data[:,2],'r-')
    plt.plot(vel_data[:,3],'g-')
    plt.title('Linear velocity measure')
    plt.legend(['x','y','z'])

    plt.figure()
    plt.plot(vel_data[:,4],'b-')
    plt.plot(vel_data[:,5],'r-')
    plt.plot(vel_data[:,6],'g-')
    plt.title('Angular velocity measure')
    plt.legend(['x','y','z'])

    plt.show()

