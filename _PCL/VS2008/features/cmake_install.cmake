# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL/features

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_features-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_features.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_features-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_features.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/features/pcl_features-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/features" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/cvfh.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/feature.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/fpfh.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/fpfh_omp.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/integral_image2D.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/integral_image_normal.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/intensity_gradient.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/intensity_spin.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/moment_invariants.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/multiscale_feature_persistence.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/narf.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/narf_descriptor.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/normal_3d.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/normal_3d_omp.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/normal_based_signature.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/pfh.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/pfhrgb.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/ppf.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/ppfrgb.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/shot.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/shot_common.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/shot_omp.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/spin_image.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/principal_curvatures.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/rift.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/rsd.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/statistical_multiscale_interest_region_extraction.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/vfh.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/3dsc.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/usc.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/boundary.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/range_image_border_extractor.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/features/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/cvfh.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/feature.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/fpfh.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/fpfh_omp.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/integral_image2D.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/integral_image_normal.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/intensity_gradient.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/intensity_spin.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/moment_invariants.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/multiscale_feature_persistence.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/narf.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/normal_3d.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/normal_3d_omp.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/normal_based_signature.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/pfh.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/pfhrgb.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/ppf.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/ppfrgb.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/shot.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/shot_common.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/shot_omp.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/spin_image.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/principal_curvatures.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/rift.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/rsd.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/statistical_multiscale_interest_region_extraction.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/vfh.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/3dsc.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/usc.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/boundary.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/range_image_border_extractor.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_range_image_border_extractor-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_range_image_border_extractor.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_range_image_border_extractor-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_range_image_border_extractor.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/features/pcl_range_image_border_extractor-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/features" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/boundary.h"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/range_image_border_extractor.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/features/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/boundary.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/features/include/pcl/features/impl/range_image_border_extractor.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/features/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

