from sot_talos_balance.control_manager                        import ControlManager
from dynamic_graph.tracer_real_time                           import TracerRealTime
from time                                                     import sleep
from sot_talos_balance.base_estimator                         import BaseEstimator
from sot_talos_balance.madgwickahrs                           import MadgwickAHRS
from sot_talos_balance.imu_offset_compensation                import ImuOffsetCompensation


from dynamic_graph.sot.core.operator import Mix_of_vector
from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from sot_talos_balance.joint_position_controller import JointPositionController
from sot_talos_balance.joint_admittance_controller import JointAdmittanceController
from sot_talos_balance.dummy_dcm_estimator import DummyDcmEstimator
from sot_talos_balance.com_admittance_controller import ComAdmittanceController
from sot_talos_balance.dcm_controller import DcmController
from sot_talos_balance.dcm_com_controller import DcmComController
from sot_talos_balance.simple_zmp_estimator import SimpleZmpEstimator

# python
from sot_talos_balance.utils.filter_utils                     import create_chebi1_checby2_series_filter
from sot_talos_balance.utils.sot_utils                        import Bunch

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

def create_imu_offset_compensation(robot, dt):
    imu_offset_compensation = ImuOffsetCompensation('imu_offset_comp');
    imu_offset_compensation.init(dt);
    plug(robot.device.accelerometer, imu_offset_compensation.accelerometer_in);
    plug(robot.device.gyrometer,     imu_offset_compensation.gyrometer_in);
    return imu_offset_compensation;

def create_device_filters(robot, dt):
    filters = Bunch();    
    filters.joints_kin    = create_chebi1_checby2_series_filter("joints_kin", dt, N_JOINTS);
    filters.ft_RF_filter  = create_chebi1_checby2_series_filter("ft_RF_filter", dt, 6);
    filters.ft_LF_filter  = create_chebi1_checby2_series_filter("ft_LF_filter", dt, 6);
    filters.acc_filter    = create_chebi1_checby2_series_filter("acc_filter", dt, 3);
    filters.gyro_filter   = create_chebi1_checby2_series_filter("gyro_filter", dt, 3);
    filters.estimator_kin = create_chebi1_checby2_series_filter("estimator_kin", dt, N_JOINTS);
    
    plug(robot.device.joint_angles,                       filters.estimator_kin.x);  # device.state, device.joint_angles or device.motor_angles ?
    plug(robot.device.forceRLEG,                          filters.ft_RF_filter.x);
    plug(robot.device.forceLLEG,                          filters.ft_LF_filter.x);
    
    # switch following lines if willing to use imu offset compensation
    #~ plug(robot.imu_offset_compensation.accelerometer_out, filters.acc_filter.x);
    plug(robot.device.accelerometer, filters.acc_filter.x);
    #~ plug(robot.imu_offset_compensation.gyrometer_out,     filters.gyro_filter.x);
    plug(robot.device.gyrometer,     filters.gyro_filter.x);
    
    return filters

def create_be_filters(robot, dt):
    be_filters = Bunch();    
    be_filters.test = create_chebi1_checby2_series_filter("test_filter", dt, N_JOINTS);
    plug(robot.base_estimator.q, be_filters.test.x);
    return be_filters
    
def create_ctrl_manager(conf, motor_params, dt, robot_name='robot'):
    ctrl_manager = ControlManager("ctrl_man");        

    ctrl_manager.tau_predicted.value    = N_JOINTS*(0.0,);
    ctrl_manager.i_measured.value       = N_JOINTS*(0.0,);
    ctrl_manager.tau_max.value          = N_JOINTS*(conf.TAU_MAX,);
    ctrl_manager.i_max.value            = N_JOINTS*(conf.CURRENT_MAX,);
    ctrl_manager.u_max.value            = N_JOINTS*(conf.CTRL_MAX,);
    
    # Init should be called before addCtrlMode 
    # because the size of state vector must be known.
    ctrl_manager.init(dt, conf.urdfFileName, robot_name)

    # Set the map from joint name to joint ID
    for key in conf.mapJointNameToID:
      ctrl_manager.setNameToId(key,conf.mapJointNameToID[key])
            
    # Set the map joint limits for each id
    for key in conf.mapJointLimits:
      ctrl_manager.setJointLimitsFromId(key,conf.mapJointLimits[key][0], \
                              conf.mapJointLimits[key][1])
          
    # Set the force limits for each id
    for key in conf.mapForceIdToForceLimits:
      ctrl_manager.setForceLimitsFromId(key,tuple(conf.mapForceIdToForceLimits[key][0]), \
                              tuple(conf.mapForceIdToForceLimits[key][1]))

    # Set the force sensor id for each sensor name
    for key in conf.mapNameToForceId:
      ctrl_manager.setForceNameToForceId(key,conf.mapNameToForceId[key])

    # Set the map from the urdf joint list to the sot joint list
    ctrl_manager.setJointsUrdfToSot(conf.urdftosot)

    # Set the foot frame name
    for key in conf.footFrameNames:
      ctrl_manager.setFootFrameName(key,conf.footFrameNames[key])
    
    # Set IMU hosting joint name
    ctrl_manager.setImuJointName(conf.ImuJointName)
    
    ctrl_manager.setRightFootForceSensorXYZ(conf.rightFootSensorXYZ);
    ctrl_manager.setRightFootSoleXYZ(conf.rightFootSoleXYZ);

    return ctrl_manager;

def create_base_estimator(robot, dt, conf, robot_name="robot"):    
    base_estimator = BaseEstimator('base_estimator');
    base_estimator.init(dt, robot_name);
    plug(robot.device.state,                      base_estimator.joint_positions);  # device.state, device.joint_angles or device.motor_angles ?
    plug(robot.device_filters.ft_LF_filter.x_filtered,   base_estimator.forceLLEG)
    plug(robot.device_filters.ft_RF_filter.x_filtered,   base_estimator.forceRLEG)
    plug(robot.device_filters.ft_LF_filter.dx,           base_estimator.dforceLLEG)
    plug(robot.device_filters.ft_RF_filter.dx,           base_estimator.dforceRLEG)
    plug(robot.device_filters.estimator_kin.dx,          base_estimator.joint_velocities);
    plug(robot.imu_filters.imu_quat,               base_estimator.imu_quaternion);   
    plug(robot.device_filters.gyro_filter.x_filtered,    base_estimator.gyroscope);
    plug(robot.device_filters.acc_filter.x_filtered,     base_estimator.accelerometer);
    base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses;
    try:
        base_estimator.w_lf_in.value = conf.w_lf_in;
        base_estimator.w_rf_in.value = conf.w_rf_in;
    except:
        pass;
    
    base_estimator.set_imu_weight(conf.w_imu);
    base_estimator.set_stiffness_right_foot(conf.K);
    base_estimator.set_stiffness_left_foot(conf.K);
    base_estimator.set_zmp_std_dev_right_foot(conf.std_dev_zmp)
    base_estimator.set_zmp_std_dev_left_foot(conf.std_dev_zmp)
    base_estimator.set_normal_force_std_dev_right_foot(conf.std_dev_fz)
    base_estimator.set_normal_force_std_dev_left_foot(conf.std_dev_fz)
    base_estimator.set_zmp_margin_right_foot(conf.zmp_margin)
    base_estimator.set_zmp_margin_left_foot(conf.zmp_margin)
    base_estimator.set_normal_force_margin_right_foot(conf.normal_force_margin)
    base_estimator.set_normal_force_margin_left_foot(conf.normal_force_margin)
    base_estimator.set_right_foot_sizes(conf.RIGHT_FOOT_SIZES)
    base_estimator.set_left_foot_sizes(conf.LEFT_FOOT_SIZES)
    

    return base_estimator;

def create_imu_filters(robot, dt):
    imu_filter = MadgwickAHRS('imu_filter');
    imu_filter.init(dt);
    plug(robot.device_filters.acc_filter.x_filtered,      imu_filter.accelerometer); # no IMU compensation
    plug(robot.device_filters.gyro_filter.x_filtered,     imu_filter.gyroscope); # no IMU compensation
    return imu_filter;
    
def addTrace(tracer, entity, signalName):
    """
    Add a signal to a tracer
    """
    signal = '{0}.{1}'.format(entity.name, signalName);
    filename = '{0}-{1}'.format(entity.name, signalName);
    tracer.add(signal, filename);
    
def addSignalsToTracer(tracer, device, outputs):
    for sign in outputs :
         addTrace(tracer,device,sign);
    return
    
def create_tracer(robot,entity,tracer_name, outputs):
    tracer = TracerRealTime(tracer_name)
    tracer.setBufferSize(80*(2**20))
    tracer.open('/tmp','dg_','.dat')
    robot.device.after.addSignal('{0}.triger'.format(tracer.name))
    addSignalsToTracer(tracer, entity, outputs)
    return tracer
	
def reset_tracer(device,tracer):
    tracer.stop();
    sleep(0.2);
    tracer.close();
    sleep(0.2);
    tracer.clear();
    sleep(0.2);
    tracer = create_tracer(device, tracer.name);
    return tracer;
    
def dump_tracer(tracer):
    tracer.stop();
    sleep(0.2);
    tracer.dump()
    sleep(0.2);
    tracer.close();

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

def create_dcm_controller(Kp,Ki,dt,robot,dcmSignal):
    from math import sqrt
    controller = DcmController("dcmCtrl")
    mass = robot.dynamic.data.mass[0]
    robot.dynamic.com.recompute(0)
    h = robot.dynamic.com.value[2]
    g = 9.81
    omega = sqrt(g/h)

    controller.Kp.value = Kp
    controller.Ki.value = Ki
    controller.decayFactor.value = 0.2
    controller.mass.value = mass
    controller.omega.value = omega

    plug(robot.dynamic.com,controller.com)
    plug(dcmSignal,controller.dcm)

    robot.dynamic.zmp.recompute(0)
    controller.zmpDes.value = robot.dynamic.zmp.value
    controller.dcmDes.value = robot.dynamic.zmp.value

    controller.init(dt)
    return controller

def create_dcm_com_controller(Kp,Ki,dt,robot,dcmSignal):
    from math import sqrt
    controller = DcmComController("dcmComCtrl")
    mass = robot.dynamic.data.mass[0]
    robot.dynamic.com.recompute(0)
    h = robot.dynamic.com.value[2]
    g = 9.81
    omega = sqrt(g/h)

    controller.Kp.value = Kp
    controller.Ki.value = Ki
    controller.decayFactor.value = 0.2
    controller.mass.value = mass
    controller.omega.value = omega

    controller.ddcomDes.value = [0.0,0.0,0.0]

    plug(dcmSignal,controller.dcm)

    robot.dynamic.com.recompute(0)
    controller.comDes.value = robot.dynamic.com.value
    controller.dcmDes.value = (robot.dynamic.com.value[0], robot.dynamic.com.value[1], 0.0)

    controller.init(dt)
    return controller

def create_parameter_server(conf, dt, robot_name='robot'):
    param_server = ParameterServer("param_server");        

    # Init should be called before addCtrlMode 
    # because the size of state vector must be known.
    param_server.init(dt, conf.urdfFileName, robot_name)

    # Set the map from joint name to joint ID
    for key in conf.mapJointNameToID:
      param_server.setNameToId(key,conf.mapJointNameToID[key])
            
    # Set the map joint limits for each id
    for key in conf.mapJointLimits:
      param_server.setJointLimitsFromId(key,conf.mapJointLimits[key][0], \
                              conf.mapJointLimits[key][1])
          
    # Set the force limits for each id
    for key in conf.mapForceIdToForceLimits:
      param_server.setForceLimitsFromId(key,tuple(conf.mapForceIdToForceLimits[key][0]), \
                              tuple(conf.mapForceIdToForceLimits[key][1]))

    # Set the force sensor id for each sensor name
    for key in conf.mapNameToForceId:
      param_server.setForceNameToForceId(key,conf.mapNameToForceId[key])

    # Set the map from the urdf joint list to the sot joint list
    param_server.setJointsUrdfToSot(conf.urdftosot)

    # Set the foot frame name
    for key in conf.footFrameNames:
      param_server.setFootFrameName(key,conf.footFrameNames[key])
    
    # Set IMU hosting joint name
    param_server.setImuJointName(conf.ImuJointName)
    
    param_server.setRightFootForceSensorXYZ(conf.rightFootSensorXYZ);
    param_server.setRightFootSoleXYZ(conf.rightFootSoleXYZ);

    return param_server;

def create_zmp_estimator(robot):
    estimator = SimpleZmpEstimator("zmpEst")
    plug(robot.dynamic.LF,estimator.poseLeft)
    plug(robot.dynamic.RF,estimator.poseRight)
    # force sensors are not mapped correctly
    plug(robot.device.forceRLEG,estimator.wrenchLeft)
    plug(robot.device.forceRARM,estimator.wrenchRight)

    estimator.init()
    return estimator

