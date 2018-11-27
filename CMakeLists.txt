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

SET(CUSTOM_HEADER_DIR "sot/talos-balance")

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
IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")
  SET(${LIBRARY_NAME}_PYTHON_FILES python/*.py)
  SET(PYTHON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}/dynamic_graph/sot/talos_balance)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIR})
ENDIF(BUILD_PYTHON_INTERFACE)

SEARCH_FOR_BOOST()

SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2")

SET(SOTTALOSBALANCE_LIB_NAME ${PROJECT_NAME})
SET(LIBRARY_NAME ${SOTTALOSBALANCE_LIB_NAME})

SET(${LIBRARY_NAME}_HEADERS
  include/sot/talos_balance/commands-helper.hh
  include/sot/talos_balance/signal-helper.hh
  include/sot/talos_balance/utils/logger.hh
  include/sot/talos_balance/utils/stop-watch.hh
  include/sot/talos_balance/utils/vector-conversions.hh
  )

#INSTALL(FILES ${${LIBRARY_NAME}_HEADERS}
#  DESTINATION include/sot/talos-balance
#  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

SET(${LIBRARY_NAME}_SOURCES ${${LIBRARY_NAME}_HEADERS}
    src/logger.cpp
    src/stop-watch.cpp
)


ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

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
  INSTALL(FILES python/dynamic_graph/sot/talos_balance/__init__.py
                python/dynamic_graph/sot/talos_balance/main.py
                DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/talos_balance)

  INSTALL(FILES python/dynamic_graph/sot/talos_balance/tests/__init__.py
                python/dynamic_graph/sot/talos_balance/tests/test_example.py
                DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/talos_balance/tests)
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(src)
# ADD_SUBDIRECTORY(unitTesting)

SETUP_PROJECT_FINALIZE()
