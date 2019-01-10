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

# Verbosity level
IF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  ADD_DEFINITIONS(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
ENDIF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

# The main include dir
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
IF(BUILD_PYTHON_INTERFACE)
  LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})
ENDIF(BUILD_PYTHON_INTERFACE)

#define DEBUG=2 if we're building in debug mode (what for?)
IF("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF ("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)

IF(UNIX)
  ADD_DEFINITIONS(-pthread)
ENDIF(UNIX)

#This project will create many plugins as shared libraries, listed here
SET(plugins
    example
    dcm-com-controller
    dcm-controller
    dummy-dcm-estimator
    com-admittance-controller
    joint-admittance-controller
    joint-position-controller
    nd-trajectory-generator
    base-estimator
    filter-differentiator
    madgwickahrs
    control-manager
    imu_offset_compensation
    parameter-server
  )

#set(ADDITIONAL_feature-task_LIBS feature-generic task)


#Plugins compilation, link, and installation
#Compiles a plugin. The plugin library is ${LIBRARY_NAME}
FOREACH(plugin ${plugins})
  #retrieve plugin name
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)

  # only one source file per plugin
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    ${plugin}.cpp)

  #remove the "lib" prefix from the plugin output name
  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    PREFIX "")

  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    SOVERSION ${PROJECT_VERSION}
    INSTALL_RPATH ${CMAKE_INSTALL_LIBDIR}/plugin)

  # Link with sot-talos-balance library
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOTTALOSBALANCE_LIB_NAME})
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

  ADD_DEPENDENCIES(${LIBRARY_NAME} ${SOTTALOSBALANCE_LIB_NAME})

  
  IF(ADDITIONAL_${LIBRARY_NAME}_LIBS)
    ADD_DEPENDENCIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
  ENDIF(ADDITIONAL_${LIBRARY_NAME}_LIBS)

  # Linux dynamic loading library flags
  IF(UNIX)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
  ENDIF(UNIX)

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl pthread)
  ENDIF(UNIX AND NOT APPLE)

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

  # build python submodule
  # (replace minus with underscore to make Python happy)
  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    SOT_TALOS_BALANCE_PYTHON_MODULE(
      ${PYTHON_LIBRARY_NAME}
      ${LIBRARY_NAME}
      sot-talos-balance-${PYTHON_LIBRARY_NAME}-wrap
      )
  ENDIF(BUILD_PYTHON_INTERFACE)
  # Install plugins
  INSTALL(TARGETS ${LIBRARY_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/plugin)
ENDFOREACH(plugin)

# Bindings Python
IF(BUILD_PYTHON_INTERFACE)
  SOT_TALOS_BALANCE_PYTHON_MODULE("" ${SOTTALOSBALANCE_LIB_NAME} wrap)

  # Install empty __init__.py files in intermediate directories.
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${SOTTALOSBALANCE_PYNAME}/__init__.py
    DESTINATION ${PYTHON_SITELIB}/${SOTTALOSBALANCE_PYNAME}
  )
ENDIF(BUILD_PYTHON_INTERFACE)
