# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL/registration

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_registration-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_registration.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_registration-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_registration.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/registration/pcl_registration-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/registration" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_estimation.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_rejection.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_rejection_distance.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_rejection_features.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_rejection_one_to_one.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_rejection_sample_consensus.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_rejection_trimmed.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_sorting.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/correspondence_types.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/ia_ransac.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/icp.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/icp_nl.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/elch.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/ndt.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/ppf_registration.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/pyramid_feature_matching.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/registration.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/transforms.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/transformation_estimation.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/transformation_estimation_svd.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/transformation_estimation_lm.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/transformation_estimation_point_to_plane.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/transformation_estimation_point_to_plane_lls.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/gicp.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/warp_point_rigid.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/warp_point_rigid_6d.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/warp_point_rigid_3d.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/distances.h"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/exceptions.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/registration/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/correspondence_estimation.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/correspondence_rejection_distance.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/correspondence_rejection_features.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/correspondence_rejection_one_to_one.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/correspondence_rejection_sample_consensus.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/correspondence_rejection_trimmed.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/correspondence_types.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/ia_ransac.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/icp.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/icp_nl.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/elch.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/ndt.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/ppf_registration.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/pyramid_feature_matching.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/registration.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/transformation_estimation_svd.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/transformation_estimation_lm.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/transformation_estimation_point_to_plane_lls.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/registration/include/pcl/registration/impl/gicp.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

