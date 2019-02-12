# Copyright 2018, Gepetto team, LAAS-CNRS
#
# This file is part of sot-talos-balance.
# sot-talos-balance is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-talos-balance is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-talos-balance. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/test.cmake)

SET(PROJECT_NAME sot-talos-balance)
SET(PROJECT_DESCRIPTION "Collection of dynamic-graph entities aimed at implementing balance control on talos.")
SET(PROJECT_URL "https://gepgitlab.laas.fr/loco-3d/sot-talos-balance")

# So that generated headers are in same place as standard ones
SET(CUSTOM_HEADER_DIR "sot/talos_balance")

SET(DOXYGEN_USE_MATHJAX YES)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
#add_compile_options(-std=c++11) # CMake 2.8.12 or newer
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

OPTION (INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" ON)
IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

PKG_CONFIG_APPEND_LIBS("sot-talos-balance")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS thread filesystem program_options unit_test_framework system regex)

OPTION (BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION (BUILD_TEST "Build dynamic-graph architecture tests" ON)

# name of the python module
IF(BUILD_PYTHON_INTERFACE)
  SET(SOTTALOSBALANCE_PYNAME sot_talos_balance)
ENDIF(BUILD_PYTHON_INTERFACE)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")
  SET(${LIBRARY_NAME}_PYTHON_FILES python/*.py)
  SET(PYTHON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME})
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIR})
ENDIF(BUILD_PYTHON_INTERFACE)

SEARCH_FOR_BOOST()

SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2")
ADD_REQUIRED_DEPENDENCY("parametric-curves")

SET(SOTTALOSBALANCE_LIB_NAME ${PROJECT_NAME})
SET(LIBRARY_NAME ${SOTTALOSBALANCE_LIB_NAME})

SET(${LIBRARY_NAME}_HEADERS
  include/sot/talos_balance/utils/logger.hh
  include/sot/talos_balance/stc-commands.hh
  include/sot/talos_balance/utils/commands-helper.hh
  include/sot/talos_balance/utils/signal-helper.hh
  include/sot/talos_balance/utils/stop-watch.hh
  include/sot/talos_balance/utils/vector-conversions.hh
  include/sot/talos_balance/utils/causal-filter.hh
  include/sot/talos_balance/utils/statistics.hh
  include/sot/talos_balance/math/fwd.hh
  include/sot/talos_balance/robot/fwd.hh
  include/sot/talos_balance/robot/robot-wrapper.hh
  )

#INSTALL(FILES ${${LIBRARY_NAME}_HEADERS}
#  DESTINATION include/sot/talos_balance
#  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

SET(${LIBRARY_NAME}_SOURCES ${${LIBRARY_NAME}_HEADERS}
    src/utils/logger.cpp    
    src/utils/causal-filter.cpp
    src/utils/statistics.cpp
    src/utils/stop-watch.cpp
    src/robot/robot-wrapper.cpp
    src/robot-utils.cpp
)


ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} parametric-curves)


IF(BUILD_PYTHON_INTERFACE)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph-python)
ENDIF(BUILD_PYTHON_INTERFACE)


IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
ENDIF(UNIX)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pthread)
ENDIF(UNIX AND NOT APPLE)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

IF(BUILD_PYTHON_INTERFACE)
  INSTALL(FILES python/${SOTTALOSBALANCE_PYNAME}/__init__.py
                python/${SOTTALOSBALANCE_PYNAME}/main.py
                python/${SOTTALOSBALANCE_PYNAME}/create_entities_utils.py
                python/${SOTTALOSBALANCE_PYNAME}/meta_task_config.py
                python/${SOTTALOSBALANCE_PYNAME}/meta_task_joint.py
                python/${SOTTALOSBALANCE_PYNAME}/control_manager_conf.py
                python/${SOTTALOSBALANCE_PYNAME}/motor_parameters.py
                DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME})
  INSTALL(FILES python/${SOTTALOSBALANCE_PYNAME}/utils/__init__.py
                python/${SOTTALOSBALANCE_PYNAME}/utils/plot_utils.py
                python/${SOTTALOSBALANCE_PYNAME}/utils/sot_utils.py
                python/${SOTTALOSBALANCE_PYNAME}/utils/filter_utils.py
                python/${SOTTALOSBALANCE_PYNAME}/utils/run_test_utils.py
                python/${SOTTALOSBALANCE_PYNAME}/utils/gazebo_utils.py
                DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}/utils)
  INSTALL(DIRECTORY python/${SOTTALOSBALANCE_PYNAME}/talos/
                DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}/talos)
  IF(BUILD_TEST)
    INSTALL(FILES python/${SOTTALOSBALANCE_PYNAME}/test/__init__.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_config_feedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_config_feedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_static_sensorfeedback.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_static_sensorfeedback.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_static_feedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_static_feedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_COMTraj_feedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_COMTraj_feedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_COMTraj_sensorfeedback.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_COMTraj_sensorfeedback.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_COMTraj_checkfeedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_COMTraj_checkfeedback_gazebo.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_ffSubscriber.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_ffSubscriber.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_zmpEstimator.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_dcmComControl.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_dcmComZmpControl.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_dcmZmpControl.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_comAdmittance.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_admittance_single_joint.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_admittance_single_joint.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_admittance_single_joint_velocity_based.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_admittance_single_joint_velocity_based.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_singleTraj.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_jointTrajGen.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_jointControl.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_COMTraj.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/appli_COMTraj.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_COMTraj_tracer.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_param_server.py
                  python/${SOTTALOSBALANCE_PYNAME}/test/test_dcm_estimator.py
                  DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}/test)
  ENDIF(BUILD_TEST)
ENDIF(BUILD_PYTHON_INTERFACE)

MACRO(DYNAMIC_GRAPH_CUSTOM_PYTHON_MODULE MODULENAME SUBMODULENAME LIBRARYNAME TARGETNAME)

  # By default the __init__.py file is installed.
  SET(INSTALL_INIT_PY 1)
  SET(SOURCE_PYTHON_MODULE "cmake/dynamic_graph/python-module-py.cc")

  IF("${SUBMODULENAME}" STREQUAL "")
    SET(FULLMODULENAME "${MODULENAME}")
  ELSE("${SUBMODULENAME}" STREQUAL "")
    SET(FULLMODULENAME "${MODULENAME}/${SUBMODULENAME}")
  ENDIF("${SUBMODULENAME}" STREQUAL "")

  # Check if there is optional parameters.
  set(extra_macro_args ${ARGN})
  list(LENGTH extra_macro_args num_extra_args)
  if( ${num_extra_args} GREATER 0)
    list(GET extra_macro_args 0 INSTALL_INIT_PY)
    if( ${num_extra_args} GREATER 1)
      list(GET extra_macro_args 1 SOURCE_PYTHON_MODULE)
    endif(${num_extra_args} GREATER 1)
  endif(${num_extra_args} GREATER 0)

  IF(NOT DEFINED PYTHONLIBS_FOUND)
    FINDPYTHON()
  ELSEIF(NOT ${PYTHONLIBS_FOUND} STREQUAL "TRUE")
    MESSAGE(FATAL_ERROR "Python has not been found.")
  ENDIF()

  SET(PYTHON_MODULE ${TARGETNAME})
  # We need to set this policy to old to accept wrap target.
  CMAKE_POLICY(PUSH)
  IF(POLICY CMP0037)
    CMAKE_POLICY(SET CMP0037 OLD)
  ENDIF()

  ADD_LIBRARY(${PYTHON_MODULE}
    MODULE
    ${PROJECT_SOURCE_DIR}/${SOURCE_PYTHON_MODULE})

  FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/${FULLMODULENAME})

  SET_TARGET_PROPERTIES(${PYTHON_MODULE}
    PROPERTIES PREFIX ""
    OUTPUT_NAME ${FULLMODULENAME}/wrap
   )
  CMAKE_POLICY(POP)

  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} "-Wl,--no-as-needed")
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} ${LIBRARYNAME} ${PYTHON_LIBRARY})

  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

  #
  # Installation
  #
  SET(PYTHON_INSTALL_DIR ${PYTHON_SITELIB}/${FULLMODULENAME})

  INSTALL(TARGETS ${PYTHON_MODULE}
    DESTINATION
    ${PYTHON_INSTALL_DIR})

  SET(ENTITY_CLASS_LIST "")
  FOREACH (ENTITY ${NEW_ENTITY_CLASS})
    SET(ENTITY_CLASS_LIST "${ENTITY_CLASS_LIST}${ENTITY}('')\n")
  ENDFOREACH(ENTITY ${NEW_ENTITY_CLASS})

  # Install if INSTALL_INIT_PY is set to 1
  IF (${INSTALL_INIT_PY} EQUAL 1)

    CONFIGURE_FILE(
      ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/submodule/__init__.py.cmake
      ${PROJECT_BINARY_DIR}/src/${FULLMODULENAME}/__init__.py
      )

    INSTALL(
      FILES ${PROJECT_BINARY_DIR}/src/${FULLMODULENAME}/__init__.py
      DESTINATION ${PYTHON_INSTALL_DIR}
      )

  ENDIF(${INSTALL_INIT_PY} EQUAL 1)

ENDMACRO(DYNAMIC_GRAPH_CUSTOM_PYTHON_MODULE)

MACRO(SOT_TALOS_BALANCE_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME)
  DYNAMIC_GRAPH_CUSTOM_PYTHON_MODULE("${SOTTALOSBALANCE_PYNAME}" "${SUBMODULENAME}" "${LIBRARYNAME}" "${TARGETNAME}")
ENDMACRO(SOT_TALOS_BALANCE_PYTHON_MODULE)

ADD_SUBDIRECTORY(src)
# ADD_SUBDIRECTORY(unittest)

SETUP_PROJECT_FINALIZE()
