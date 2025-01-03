########## MACROS ###########################################################################
#############################################################################################

# Requires CMake > 3.15
if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeDeps' generator only works with CMake >= 3.15")
endif()

if(osqp_FIND_QUIETLY)
    set(osqp_MESSAGE_MODE VERBOSE)
else()
    set(osqp_MESSAGE_MODE STATUS)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmakedeps_macros.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/osqpTargets.cmake)
include(CMakeFindDependencyMacro)

check_build_type_defined()

foreach(_DEPENDENCY ${osqp_FIND_DEPENDENCY_NAMES} )
    # Check that we have not already called a find_package with the transitive dependency
    if(NOT ${_DEPENDENCY}_FOUND)
        find_dependency(${_DEPENDENCY} REQUIRED ${${_DEPENDENCY}_FIND_MODE})
    endif()
endforeach()

set(osqp_VERSION_STRING "0.6.3")
set(osqp_INCLUDE_DIRS ${osqp_INCLUDE_DIRS_RELEASE} )
set(osqp_INCLUDE_DIR ${osqp_INCLUDE_DIRS_RELEASE} )
set(osqp_LIBRARIES ${osqp_LIBRARIES_RELEASE} )
set(osqp_DEFINITIONS ${osqp_DEFINITIONS_RELEASE} )

# Only the first installed configuration is included to avoid the collision
foreach(_BUILD_MODULE ${osqp_BUILD_MODULES_PATHS_RELEASE} )
    message(${osqp_MESSAGE_MODE} "Conan: Including build module from '${_BUILD_MODULE}'")
    include(${_BUILD_MODULE})
endforeach()


