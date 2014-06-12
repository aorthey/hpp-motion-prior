# Copyright (c) 2014 CNRS
# Author: Andreas Orthey
#
# This file is part of hpp-motion-prior.
# hpp-motion-prior is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-motion-prior is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

SET(IDL_SOURCES
  precomputation
)

OMNIIDL_INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/idl)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_FILE (${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/motion-prior)
ENDFOREACH()

SET(LIBRARY_NAME ${PROJECT_NAME})
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  server.cc
  constraint-manifold-operator.cc
  constraint-manifold-operator.hh
  irreducible-configuration-shooter.hh
  capsule-parser.cc
  capsule-parser.hh
  precomputation.impl.cc
  precomputation.impl.hh
  precomputation-utils.cc
  precomputation-utils.hh
  precomputationSK.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/motion_prior/precomputation_idl.py
)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model-urdf)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-template-corba)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

# search for python
FINDPYTHON()

# search for omniidl
FIND_PROGRAM(OMNIIDL omniidl)
IF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)
  MESSAGE(FATAL_ERROR "cannot find omniidl.")
ENDIF(${OMNIIDL} STREQUAL OMNIIDL-NOTFOUND)

FOREACH (IDL ${IDL_SOURCES})
  ADD_CUSTOM_COMMAND(
    OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/motion_prior/${IDL}_idl.py
    COMMAND ${OMNIIDL} ARGS -bpython -Wbpackage=""
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/motion-prior/${IDL}.idl
    MAIN_DEPENDENCY
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/motion-prior/${IDL}.idl
    )
  SET_SOURCE_FILES_PROPERTIES(
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/motion_prior/${IDL}_idl.py
    GENERATED=ON
    )
  INSTALL(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/motion_prior
    )
ENDFOREACH()

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/motion_prior/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/motion_prior
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/motion_prior/client.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/motion_prior
  )

ADD_EXECUTABLE (hpp-precomputation-corba-server hpp-precomputation-corba-server.cc)
TARGET_LINK_LIBRARIES (   hpp-precomputation-corba-server ${LIBRARY_NAME})
PKG_CONFIG_USE_DEPENDENCY(hpp-precomputation-corba-server hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(hpp-precomputation-corba-server hpp-template-corba)
PKG_CONFIG_USE_DEPENDENCY(hpp-precomputation-corba-server hpp-util)

INSTALL (TARGETS hpp-precomputation-corba-server DESTINATION ${CMAKE_INSTALL_BINDIR})
