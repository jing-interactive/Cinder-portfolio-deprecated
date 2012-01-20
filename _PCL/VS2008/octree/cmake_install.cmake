# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL/octree

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_octree-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_octree.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_octree-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_octree.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/octree/pcl_octree-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/octree" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_base.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_impl.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_nodes.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_pointcloud_density.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_pointcloud_occupancy.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_pointcloud_singlepoint.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_pointcloud_pointvector.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_pointcloud_changedetector.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_pointcloud_voxelcentroid.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_pointcloud.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_iterator.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_search.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree2buf_base.h"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/octree_lowmemory_base.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/octree/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/impl/octree_base.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/impl/octree_pointcloud.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/impl/octree2buf_base.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/impl/octree_lowmemory_base.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/impl/octree_iterator.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/octree/include/pcl/octree/impl/octree_search.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/octree/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

