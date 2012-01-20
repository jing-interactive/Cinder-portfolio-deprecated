# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL/common

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_common-gd.lib")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/lib/pcl_common.lib")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_common-gd.dll")
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/bin/pcl_common.dll")
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/common/pcl_common-1.4.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/correspondence.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/exceptions.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/pcl_base.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/pcl_macros.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/point_cloud.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/point_traits.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/point_types_conversion.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/point_representation.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/correspondence.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/point_types.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/win32_macros.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/cloud_properties.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/channel_properties.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/for_each_type.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/pcl_tests.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/ModelCoefficients.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/PolygonMesh.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/Vertices.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/PointIndices.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/TextureMesh.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/../sensor_msgs" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/sensor_msgs/PointField.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/sensor_msgs/PointCloud2.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/sensor_msgs/Image.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/../std_msgs" TYPE FILE FILES "f:/__svn_pool/CreativeCoding/_PCL/common/include/std_msgs/Header.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/common" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/angles.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/bivariate_polynomial.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/centroid.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/concatenate.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/common.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/common_headers.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/distances.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/eigen.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/io.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/file_io.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/intersections.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/norms.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/point_correspondence.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/piecewise_linear_function.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/polynomial_calculations.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/poses_from_matches.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/time.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/time_trigger.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/transforms.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/transformation_from_correspondences.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/vector_average.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/pca.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/synchronizer.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/utils.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/geometry.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/gaussian.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/spring.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/point_operators.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/convolution.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/common/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/angles.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/bivariate_polynomial.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/centroid.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/common.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/eigen.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/io.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/file_io.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/norms.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/piecewise_linear_function.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/polynomial_calculations.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/pca.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/transforms.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/transformation_from_correspondences.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/vector_average.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/gaussian.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/spring.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/common/impl/convolution.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/impl" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/impl/instantiate.hpp"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/impl/point_types.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/ros" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/ros/conversions.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/ros/register_point_struct.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/ros/for_each_type.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/ros/point_traits.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl/console" TYPE FILE FILES
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/console/parse.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/console/print.h"
    "f:/__svn_pool/CreativeCoding/_PCL/common/include/pcl/console/time.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/common/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

