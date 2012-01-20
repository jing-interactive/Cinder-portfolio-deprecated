# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL/surface

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "C:/Program Files/PCL")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_surface-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_surface.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_surface-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_surface.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/surface/pcl_surface-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/surface" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/ear_clipping.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/gp3.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/grid_projection.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/marching_cubes.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/marching_cubes_greedy.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/marching_cubes_greedy_dot.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/mls.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/mls_omp.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/organized_fast_mesh.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/reconstruction.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/processing.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/simplification_remove_unused_vertices.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/surfel_smoothing.h"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/texture_mapping.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/surface/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/gp3.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/grid_projection.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/marching_cubes.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/marching_cubes_greedy.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/marching_cubes_greedy_dot.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/mls.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/mls_omp.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/organized_fast_mesh.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/reconstruction.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/surfel_smoothing.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/surface/include/pcl/surface/impl/texture_mapping.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "surface")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/surface/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

