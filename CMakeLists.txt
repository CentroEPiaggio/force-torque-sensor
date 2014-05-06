cmake_minimum_required(VERSION 2.8)
project(FTSensor)

set(FTSENSOR_MAJOR_VERSION 0)
set(FTSENSOR_MINOR_VERSION 1)
set(FTSENSOR_PATCH_VERSION 0)
set(FTSENSOR_VERSION
  ${FTSENSOR_MAJOR_VERSION}.${FTSENSOR_MINOR_VERSION}.${FTSENSOR_PATCH_VERSION})

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/FTSensor)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

find_package(LibXml2 REQUIRED)

# set up include-directories
include_directories(
  "${PROJECT_SOURCE_DIR}/class/include"   # to find foo/foo.h
  "${PROJECT_BINARY_DIR}")  # to find foo/config.h

include_directories(
	${LIBXML2_INCLUDE_DIR}
)

# Add sub-directories
add_subdirectory(class)
add_subdirectory(examples)

# The interesting stuff goes here
# ===============================

# Add all targets to the build-tree export set
export(TARGETS FTSensor
  FILE "${PROJECT_BINARY_DIR}/FTSensorTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE FTSensor)

# Create the FTSensorConfig.cmake and FTSensorConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(FTSensorConfig.cmake.in
  "${PROJECT_BINARY_DIR}/FTSensorConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${FTSENSOR_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(FTSensorConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FTSensorConfig.cmake" @ONLY)
# ... for both
configure_file(FTSensorConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/FTSensorConfigVersion.cmake" @ONLY)

# Install the FTSensorConfig.cmake and FTSensorConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FTSensorConfig.cmake"
  "${PROJECT_BINARY_DIR}/FTSensorConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT FTSensorTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

install(FILES class/include/FTSensor/FTSensor.h DESTINATION  ${INSTALL_INCLUDE_DIR}/FTSensor)

###############################################################################
#
# ROS
#
###############################################################################

  # we need the ROS version to be hydro
SET(ROS_DIRECTORY /opt/ros/hydro CACHE PATH "ROS directory")
SET(CATKIN_DIRECTORY ${PROJECT_BINARY_DIR}/catkin CACHE PATH "Catkin workspace directory")  
option(BUILD_ROS_PKGS "Build ROS packages" NO)
mark_as_advanced(BUILD_ROS_PKGS)

if(BUILD_ROS_PKGS)
  if(DEFINED ENV{ROS_DISTRO}) 
    if($ENV{ROS_DISTRO} STREQUAL "hydro")
      MESSAGE(STATUS "current ROS_DISTRO: $ENV{ROS_DISTRO}")
      # catkin workspace environment configuration
      if(NOT(EXISTS "${PROJECT_BINARY_DIR}/catkin"))
        FILE(MAKE_DIRECTORY ${CATKIN_DIRECTORY})  
        FILE(MAKE_DIRECTORY ${CATKIN_DIRECTORY}/src) 
        EXECUTE_PROCESS(COMMAND catkin_init_workspace ${CATKIN_DIRECTORY}/src)
        EXECUTE_PROCESS(COMMAND catkin_make -C ${CATKIN_DIRECTORY})
        # this is not working as expected, needs to be done outside
        #EXECUTE_PROCESS(COMMAND source ${CATKIN_DIRECTORY}/setup.bash)
        EXECUTE_PROCESS(COMMAND ln -s ${PROJECT_SOURCE_DIR}/ros/ftsensor ${CATKIN_DIRECTORY}/src)
      endif()
      # build and rebuild re-build the packages after all desired targets are built
      ADD_CUSTOM_TARGET(ROSPkgs ALL catkin_make -C ${CATKIN_DIRECTORY}
        COMMENT "Built target ROSPkgs"
        DEPENDS FTSensor
      )
    endif()   
  else()
    MESSAGE(SEND_ERROR "You enabled BUILD_ROS_PKGS, but this option can not be processed because:\n1. ROS system is not installed. Recall to install the hydro distribution.\n2. The ROS environment is not loaded, so ensure you type in this terminal: source /opt/ros/hydro/setup.bash\n3. Loaded ROS distribution is not hydro.\n" )    
  endif()
endif()


# find_package(Doxygen)
# if(DOXYGEN_FOUND)
#	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in @ONLY)
#	add_custom_target(doc
#	${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in
#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#	COMMENT "Generating API documentation with Doxygen" VERBATIM)
# endif(DOXYGEN_FOUND)