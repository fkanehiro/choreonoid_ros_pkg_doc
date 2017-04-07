#
#
#

cmake_minimum_required(VERSION 2.8.3)
cmake_policy(VERSION ${CMAKE_MINIMUM_REQUIRED_VERSION})

if (CMAKE_VERSION VERSION_GREATER "2.9.9.9")
  cmake_policy(SET CMP0037 OLD)
endif()

project(choreonoid_plugins)

# NOTE: if you chosen the CMake build type with not set optimize option of compiler,
# the depth camera device process very slowly.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS 
  choreonoid_ros
  message_generation 
  roscpp 
  rospy 
  nodelet 
  angles 
  std_srvs 
  gazebo_msgs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  tf 
  tf2_ros 
  dynamic_reconfigure 
  driver_base 
  rosgraph_msgs 
  trajectory_msgs 
  image_transport 
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
)

include (FindPkgConfig)
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
if (PKG_CONFIG_FOUND)
  pkg_check_modules(CNOID REQUIRED choreonoid)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir choreonoid
                    OUTPUT_VARIABLE CNOID_PLUGIN_SUBDIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0" "1.46.1" "1.47" "1.47.0" "1.48" "1.48.0" "1.49.0" "1.50.0")
set(boost_packages system filesystem program_options regex thread iostreams date_time)
find_package(Boost 1.36.0 REQUIRED COMPONENTS ${boost_packages})

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CNOID_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(
  ${CNOID_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

catkin_package(
  LIBRARIES
  CnoidRosBodyPlugin
  INCLUDE_DIRS
  src
  CATKIN_DEPENDS
  message_generation 
  roscpp 
  rospy 
  nodelet 
  angles 
  std_srvs 
  gazebo_msgs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  tf 
  tf2_ros
  dynamic_reconfigure 
  driver_base 
  rosgraph_msgs 
  trajectory_msgs 
  image_transport 
  rosconsole
  camera_info_manager
)

option(ENABLE_PYTHON "" ON)

if (CMAKE_VERSION VERSION_LESS "3.1")
  if (CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND g++ -dumpversion OUTPUT_VARIABLE gxxver)

   if (gxxver VERSION_LESS "4.7")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   else()
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   endif()
  else()
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  endif()
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -finline-functions")
option(ENABLE_GCC_FVISIBILITY_HIDDEN "Use the -fvisibility=hidden option when the sources are compiled" ON)
option(ENABLE_INSTALL_RPATH "Enable RPATH setting for installed binary files" ON)

function(add_cnoid_plugin)

  add_library(${ARGV})

  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
    if(compile_flags)
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
    else()
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
    endif()
  endif()

#  set_target_properties(${ARGV0} PROPERTIES
#    LIBRARY_OUTPUT_DIRECTORY ${CNOID_PLUGIN_SUBDIR}
#    ARCHIVE_OUTPUT_DIRECTORY ${CNOID_PLUGIN_SUBDIR}
#    RUNTIME_OUTPUT_DIRECTORY ${CNOID_PLUGIN_SUBDIR})

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "@rpath")
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path,-rpath,@loader_path/..")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
    endif()
  else()
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
  endif()

endfunction()

if(ENABLE_PYTHON)

  find_package(PythonLibs 2.7 REQUIRED)
  include_directories(${PYTHON_INCLUDE_PATH})
  set(CNOID_PYTHON_SUBDIR ${CNOID_PLUGIN_SUBDIR}/python)

  function(add_cnoid_python_module)
  
    set(target ${ARGV0})
    string(REGEX REPLACE "^Py(.+)$" "\\1" module ${target})
    set(sources ${ARGV})
    list(REMOVE_AT sources 0)

    add_library(${target} SHARED ${sources})
    
    if(NOT WIN32)
      set_target_properties(${target}  PROPERTIES
      COMPILE_DEFINITIONS "BOOST_PYTHON_USE_GCC_SYMBOL_VISIBILITY" )
    endif()

    set_target_properties(${target}  PROPERTIES
    #  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR}/cnoid
    #  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR}/cnoid
      RUNTIME_OUTPUT_NAME ${module}
      LIBRARY_OUTPUT_NAME ${module}
      PREFIX "")

    #install(TARGETS ${target}
    #  RUNTIME DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
    #  LIBRARY DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

  endfunction()


endif()

## Plugins
add_cnoid_plugin(CnoidRosBodyPlugin
  SHARED
  src/RosBodyPlugin.cpp
  src/BodyRosItem.cpp
  src/WorldRosItem.cpp
  src/BodyRosJointControllerItem.cpp
  src/BodyRosTorqueControllerItem.cpp
  src/BodyRosHighgainControllerItem.cpp
)

target_link_libraries(CnoidRosBodyPlugin ${CNOID_LIBRARIES} CnoidBodyPlugin ${catkin_LIBRARIES})

install(TARGETS 
  CnoidRosBodyPlugin
  RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}
  LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}
  ARCHIVE DESTINATION ${CNOID_PLUGIN_SUBDIR}
)

install(
  DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.h"
  PATTERN "choreonoid_plugins" EXCLUDE
  PATTERN "python" EXCLUDE
)

install(
  DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(ENABLE_PYTHON)
  add_subdirectory(src/python)
endif()
