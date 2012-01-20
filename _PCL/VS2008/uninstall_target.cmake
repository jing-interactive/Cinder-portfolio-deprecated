if(NOT EXISTS "F:/__svn_pool/CreativeCoding/_PCL/VS2008/install_manifest.txt")
    message(FATAL_ERROR "Cannot find install manifest: \"F:/__svn_pool/CreativeCoding/_PCL/VS2008/install_manifest.txt\"")
endif(NOT EXISTS "F:/__svn_pool/CreativeCoding/_PCL/VS2008/install_manifest.txt")

file(READ "F:/__svn_pool/CreativeCoding/_PCL/VS2008/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
    message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
    message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
    if(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
        exec_program("D:/Program Files/CMake 2.8/bin/cmake.exe" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
            OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
        if(NOT "${rm_retval}" STREQUAL 0)
            message(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
        endif(NOT "${rm_retval}" STREQUAL 0)
    else(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
        message(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
    endif(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
endforeach(file)

# remove pcl directory in include (removes all files in it!)
message(STATUS "Uninstalling \"C:/Program Files/PCL/include/pcl-1.4\"")
if(EXISTS "C:/Program Files/PCL/include/pcl-1.4")
    exec_program("D:/Program Files/CMake 2.8/bin/cmake.exe"
        ARGS "-E remove_directory \"C:/Program Files/PCL/include/pcl-1.4\""
        OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR
            "Problem when removing \"C:/Program Files/PCL/include/pcl-1.4\"")
    endif(NOT "${rm_retval}" STREQUAL 0)
else(EXISTS "C:/Program Files/PCL/include/pcl-1.4")
    message(STATUS
        "Directory \"C:/Program Files/PCL/include/pcl-1.4\" does not exist.")
endif(EXISTS "C:/Program Files/PCL/include/pcl-1.4")

# remove pcl directory in share (removes all files in it!)
# created by CMakeLists.txt for PCLConfig.cmake
message(STATUS "Uninstalling \"C:/Program Files/PCL/cmake\"")
if(EXISTS "C:/Program Files/PCL/cmake")
    exec_program("D:/Program Files/CMake 2.8/bin/cmake.exe"
        ARGS "-E remove_directory \"C:/Program Files/PCL/cmake\""
        OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR
            "Problem when removing \"C:/Program Files/PCL/cmake\"")
    endif(NOT "${rm_retval}" STREQUAL 0)
else(EXISTS "C:/Program Files/PCL/cmake")
    message(STATUS
        "Directory \"C:/Program Files/PCL/cmake\" does not exist.")
endif(EXISTS "C:/Program Files/PCL/cmake")
