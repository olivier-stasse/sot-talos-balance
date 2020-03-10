# Copyright 2018, Gepetto team, LAAS-CNRS


SET(plugins
  example
  delay
  boolean-identity
  round-double-to-int
  int-identity
  pose-quaternion-to-matrix-homo
  euler-to-quat
  quat-to-euler
  state-transformation
  simple-reference-frame
  simple-state-integrator
  dummy-walking-pattern-generator
  simple-zmp-estimator
  simple-distribute-wrench
  simple-pid
  simple-pidd
  simple-controller-6d
  distribute-wrench
  dcm-com-controller
  dcm-controller
  dummy-dcm-estimator
  com-admittance-controller
  simple-admittance-controller
  coupled-admittance-controller
  admittance-controller-end-effector
  ankle-admittance-controller
  foot-force-difference-controller
  joint-position-controller
  nd-trajectory-generator
  talos-base-estimator
  talos-control-manager
  dcm-estimator
  qualisys-client
  ft-calibration
  ft-wrist-calibration
  ankle-joint-selector
  saturation
  hip-flexibility-compensation
  )

# set(feature-task_deps feature-generic task)


FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${plugin})

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${PROJECT_NAME} ${${LIBRARY_NAME}_deps})

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    SOT_TALOS_BALANCE_PYTHON_MODULE(${PYTHON_LIBRARY_NAME}
      ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap)
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

IF(BUILD_PYTHON_INTERFACE)
  SOT_TALOS_BALANCE_PYTHON_MODULE("" ${PROJECT_NAME} wrap)

  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PY_NAME}/__init__.py
    DESTINATION ${PYTHON_SITELIB}/${PY_NAME}
    )
ENDIF(BUILD_PYTHON_INTERFACE)
