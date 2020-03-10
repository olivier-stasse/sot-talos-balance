# main python module
SET(${PROJECT_NAME}_PYTHON_FILES
  __init__.py
  main.py
  create_entities_utils.py
  meta_task_config.py
  meta_task_joint.py
  meta_task_pose.py
  motor_parameters.py
  )

FOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PY_NAME} ${python})
ENDFOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})

# utils submodule
SET(${PROJECT_NAME}_UTILS_PYTHON_FILES
  __init__.py
  control_utils.py
  plot_utils.py
  sot_utils.py
  filter_utils.py
  run_test_utils.py
  gazebo_utils.py
  )

FOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PY_NAME}/utils ${python})
ENDFOREACH(python ${${PROJECT_NAME}_UTILS_PYTHON_FILES})


# talos submodule
SET(${PROJECT_NAME}_TALOS_PYTHON_FILES
  __init__.py
  admittance_ctrl_conf.py
  balance_ctrl_conf.py
  balance_ctrl_sim_conf.py
  base_estimator_conf.py
  base_estimator_sim_conf.py
  distribute_conf.py
  control_manager_conf.py
  control_manager_sim_conf.py
  current_controller_conf.py
  current_controller_sim_conf.py
  force_offset_conf.py
  force_torque_estimator_conf.py
  ft_calibration_conf.py
  ft_wrist_calibration_conf.py
  joint_torque_controller_conf.py
  joint_torque_controller_sim_conf.py
  parameter_server_conf.py
  hip_flexibility_compensation_conf.py
  )

FOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PY_NAME}/talos ${python})
ENDFOREACH(python ${${PROJECT_NAME}_TALOS_PYTHON_FILES})

# simulation files
SET(${PROJECT_NAME}_SIMULATION_FILES
  __init__.py
  test_dcm_zmp_control.py
  appli_dcm_zmp_control.py
  test_dcm_zmp_control_distribute.py
  appli_dcm_zmp_control_distribute.py
  test_dcm_zmp_control_ffdc.py
  appli_dcm_zmp_control_ffdc.py
  test_ffSubscriber.py
  appli_ffSubscriber.py
  test_zmpEstimator.py
  appli_zmpEstimator.py
  test_dcmComControl.py
  appli_dcmComControl.py
  test_dcmComZmpControl.py
  appli_dcmComZmpControl.py
  test_dcmZmpControl.py
  appli_dcmZmpControl.py
  test_dcmZmpControl_file.py
  appli_dcmZmpControl_file.py
  test_comAdmittance.py
  test_admittance_single_joint.py
  appli_admittance_single_joint.py
  test_admittance_end_effector.py
  appli_admittance_end_effector.py
  test_simple_ankle_admittance.py
  appli_simple_ankle_admittance.py
  test_admittance_single_joint_velocity_based.py
  appli_admittance_single_joint_velocity_based.py
  test_singleTraj.py
  test_jointTrajGen.py
  test_jointControl.py
  test_COMTraj.py
  appli_COMTraj.py
  test_COMTraj_tracer.py
  test_param_server.py
  test_dcm_estimator.py
  appli_dcm_estimator.py
  test_hip_flexibility_compensation.py
  appli_hip_flexibility_compensation.py
  )

FOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})
  PYTHON_INSTALL_ON_SITE(${PY_NAME}/test ${python})
ENDFOREACH(python ${${PROJECT_NAME}_SIMULATION_FILES})
