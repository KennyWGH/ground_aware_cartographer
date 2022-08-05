# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(cartographer_ros)

set(PACKAGE_DEPENDENCIES
  cartographer_ros_msgs
  geometry_msgs
  message_runtime
  nav_msgs
  pcl_conversions
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)

if(WIN32)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()
# For yet unknown reason, if Boost is find_packaged() after find_package(cartographer),
# some Boost libraries including Thread are set to static, despite Boost_USE_STATIC_LIBS,
# which causes linking problems on windows due to shared/static Thread clashing.
# PCL also finds Boost. Work around by moving before find_package(cartographer).
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(PCL REQUIRED COMPONENTS common)

if(${USE_THIRDPARTY_CARTOGRAPHER}) 
  message(STATUS "************************************************************************* ")
  message(STATUS "***************** Going to use thirdparty cartographer! ***************** ")
  message(STATUS "************************************************************************* ")
  string(REGEX REPLACE "(.*)/(.*)" "\\1" CMAKE_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  message(STATUS "CMAKE PARENT DIR: ${CMAKE_PARENT_DIR}")
  set(cartographer_DIR ${CMAKE_PARENT_DIR}/cartographer/share/cartographer)
  message(STATUS "cartographer DIR: ${cartographer_DIR}")
  message(STATUS "************************************************************************* ")
endif()

# option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
# include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions_for_cartographer_ros.cmake")
# google_initialize_cartographer_ros_project()

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()

if(${cartographer_FOUND})
  message(STATUS "########################")
  message(STATUS "Found cartographer dir: ${cartographer_DIR}")
  message(STATUS "########################")
endif()

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include(FindPkgConfig)

find_package(absl REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  ${urdfdom_headers_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    # TODO(damonkohler): This should be here but causes Catkin to abort because
    # protobuf specifies a library '-lpthread' instead of just 'pthread'.
    # CARTOGRAPHER
    PCL
    EIGEN3
    Boost
    urdfdom_headers
  INCLUDE_DIRS "."
  LIBRARIES ${PROJECT_NAME}
)

file(GLOB_RECURSE ALL_SRCS "cartographer_ros/*.cc" "cartographer_ros/*.h")
file(GLOB_RECURSE ALL_TESTS "cartographer_ros/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer_ros/*_main.cc")
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer_ros/cartographer_grpc/*")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
add_subdirectory("cartographer_ros")

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# Catkin
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(DIRECTORY launch urdf configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tf_remove_frames.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install source headers.
file(GLOB_RECURSE HDRS "cartographer_ros/*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

# 如果使用第三方cartographer库，需要单独安装该库的配置文件。
if(${USE_THIRDPARTY_CARTOGRAPHER}) 
  message(STATUS "************************************************************************* ")
  message(STATUS "************ installing thirdparty cartographer config files! *********** ")
  message(STATUS "************************************************************************* ")
  message(STATUS "cmake curr source dir: ${CMAKE_CURRENT_SOURCE_DIR}")
  message(STATUS "cmake install prefix: ${CMAKE_INSTALL_PREFIX}")
  if(NOT CMAKE_PARENT_DIR)
    string(REGEX REPLACE "(.*)/(.*)" "\\1" CMAKE_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
    message(STATUS "CMAKE PARENT DIR: ${CMAKE_PARENT_DIR}")
  endif()
  set(CARTO_CONFIG_FILES_DIR ${CMAKE_PARENT_DIR}/cartographer/share/cartographer/configuration_files)
  # set(CARTO_CONFIG_FILES_DIR ${CMAKE_PARENT_DIR}/cartographer_rviz/ogre_media/materials/glsl120)
  message(STATUS "config files dir: ${CARTO_CONFIG_FILES_DIR}")
  install(DIRECTORY ${CARTO_CONFIG_FILES_DIR} DESTINATION share/cartographer/)
  message(STATUS "************************************************************************* ")
endif()
