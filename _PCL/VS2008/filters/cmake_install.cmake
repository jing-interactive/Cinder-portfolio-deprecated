# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL/filters

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_filters-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_filters.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_filters-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_filters.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/filters/pcl_filters-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/filters" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/conditional_removal.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/crop_box.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/clipper3D.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/plane_clipper3D.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/crop_hull.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/extract_indices.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/filter.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/filter_indices.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/passthrough.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/project_inliers.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/radius_outlier_removal.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/random_sample.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/normal_space.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/statistical_outlier_removal.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/voxel_grid.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/approximate_voxel_grid.h"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/bilateral.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/filters/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/conditional_removal.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/crop_box.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/crop_hull.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/plane_clipper3D.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/extract_indices.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/filter.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/filter_indices.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/passthrough.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/project_inliers.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/radius_outlier_removal.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/random_sample.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/normal_space.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/statistical_outlier_removal.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/voxel_grid.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/approximate_voxel_grid.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/filters/include/pcl/filters/impl/bilateral.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/filters/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

