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

#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    find_package(cartographer REQUIRED)
#    target_link_libraries(MY_TARGET_NAME PUBLIC cartographer)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was cartographer-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set_and_check(CARTOGRAPHER_CMAKE_DIR "${PACKAGE_PREFIX_DIR}/share/cartographer/cmake")

set(CERES_DIR_HINTS /usr/local/lib/cmake/Ceres)
set(CARTOGRAPHER_HAS_GRPC OFF)

if (cartographer_FIND_QUIETLY)
   set(QUIET_OR_REQUIRED_OPTION "QUIET")
elseif (cartographer_FIND_REQUIRED)
   set(QUIET_OR_REQUIRED_OPTION "REQUIRED")
else ()
   set(QUIET_OR_REQUIRED_OPTION "")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CARTOGRAPHER_CMAKE_DIR}/modules)

find_package(Ceres ${QUIET_OR_REQUIRED_OPTION} HINTS ${CERES_DIR_HINTS})
if (WIN32)
    find_package(glog REQUIRED)
endif()
find_package(absl ${QUIET_OR_REQUIRED_OPTION})
if(CARTOGRAPHER_HAS_GRPC)
   find_package(async_grpc ${QUIET_OR_REQUIRED_OPTION})
endif()

include("${CARTOGRAPHER_CMAKE_DIR}/CartographerTargets.cmake")

unset(QUIET_OR_REQUIRED_OPTION)
