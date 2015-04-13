# Copyright (c) 2014 LAAS-CNRS
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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-motion-prior)
SET(PROJECT_DESCRIPTION
        "Precomputation of Mechanical System and Evironment before Motion
        Planning is conducted")
SET(PROJECT_URL "https://github.com/orthez/hpp-motion-prior")
SET(PROJECT_VERSION "0.2 alpha-experimental")

SET(CMAKE_INSTALL_PREFIX "/home/ijrr-2015/devel/hpp-stable/install")

MESSAGE("${CMAKE_INSTALL_PREFIX}")
SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/motion-prior/server.hh
  include/hpp/corbaserver/motion-prior/irreducible-configuration-shooter.hh
  include/hpp/corbaserver/motion-prior/fwd.hh
)
SET(CUSTOM_HEADER_DIR hpp/corbaserver/motion-prior)

SETUP_PROJECT()
# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

MESSAGE("${HEADER_FILES}")

ADD_DOC_DEPENDENCY ("hpp-core >= 3")
#ADD_REQUIRED_DEPENDENCY("hpp-model-urdf >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-fcl >= 0")
ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 2.1")
ADD_REQUIRED_DEPENDENCY("hpp-model-urdf >= 3")
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")
ADD_REQUIRED_DEPENDENCY("hpp-template-corba >= 1.1")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")

IF (NOT ${HPP_CORBASERVER_PREFIX} STREQUAL ${CMAKE_INSTALL_PREFIX})
  MESSAGE(WARNING
    "  THIS PACKAGE SHOULD BE INSTALLED IN THE SAME PREFIX AS HPP-CORBASERVER,\n"
    "  SINCE IT INSTALLS A PYTHON SUBMODULE OF HPP.CORBASERVER.")
  MESSAGE("CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
  MESSAGE("HPP_CORBASERVER_PREFIX=${HPP_CORBASERVER_PREFIX}")
ENDIF()

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
PKG_CONFIG_APPEND_LIBS("hpp-motion-prior")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
##################################
