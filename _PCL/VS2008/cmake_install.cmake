# Install script for directory: f:/__svn_pool/CreativeCoding/_PCL

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.4/pcl" TYPE FILE FILES "F:/__svn_pool/CreativeCoding/_PCL/VS2008/include/pcl/pcl_config.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "pclconfig")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES
    "F:/__svn_pool/CreativeCoding/_PCL/VS2008/PCLConfig.cmake"
    "F:/__svn_pool/CreativeCoding/_PCL/VS2008/PCLConfigVersion.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "pclconfig")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE PROGRAM FILES
    "d:/Program Files/Microsoft Visual Studio 9.0/VC/redist/x86/Microsoft.VC90.CRT/Microsoft.VC90.CRT.manifest"
    "d:/Program Files/Microsoft Visual Studio 9.0/VC/redist/x86/Microsoft.VC90.CRT/msvcm90.dll"
    "d:/Program Files/Microsoft Visual Studio 9.0/VC/redist/x86/Microsoft.VC90.CRT/msvcp90.dll"
    "d:/Program Files/Microsoft Visual Studio 9.0/VC/redist/x86/Microsoft.VC90.CRT/msvcr90.dll"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/common/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/kdtree/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/octree/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/search/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/range_image/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/features/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/sample_consensus/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/filters/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/io/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/keypoints/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/registration/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/segmentation/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/surface/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/test/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/visualization/cmake_install.cmake")
  INCLUDE("F:/__svn_pool/CreativeCoding/_PCL/VS2008/tools/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "F:/__svn_pool/CreativeCoding/_PCL/VS2008/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "F:/__svn_pool/CreativeCoding/_PCL/VS2008/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
