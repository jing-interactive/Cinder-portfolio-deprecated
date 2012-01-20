# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL/sample_consensus

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_sample_consensus-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_sample_consensus.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_sample_consensus-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_sample_consensus.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/sample_consensus/pcl_sample_consensus-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/sample_consensus" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/lmeds.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/method_types.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/mlesac.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/model_types.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/msac.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/ransac.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/rmsac.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/rransac.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_circle.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_cylinder.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_line.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_stick.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_normal_parallel_plane.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_normal_plane.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_line.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_plane.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_perpendicular_plane.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_plane.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_registration.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/sac_model_sphere.h"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/prosac.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/sample_consensus/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/lmeds.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/mlesac.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/msac.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/ransac.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/rmsac.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/rransac.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_circle.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cylinder.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_line.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_stick.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_plane.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_line.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_plane.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_plane.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_registration.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/sac_model_sphere.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/sample_consensus/include/pcl/sample_consensus/impl/prosac.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

