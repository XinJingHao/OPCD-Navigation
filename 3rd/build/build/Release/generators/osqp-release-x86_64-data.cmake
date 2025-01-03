########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

set(osqp_COMPONENT_NAMES "")
set(osqp_FIND_DEPENDENCY_NAMES "")

########### VARIABLES #######################################################################
#############################################################################################
set(osqp_PACKAGE_FOLDER_RELEASE "/home/xjh/.conan/data/osqp/0.6.3/_/_/package/6af9cc7cb931c5ad942174fd7838eb655717c709")
set(osqp_BUILD_MODULES_PATHS_RELEASE )


set(osqp_INCLUDE_DIRS_RELEASE "${osqp_PACKAGE_FOLDER_RELEASE}/include")
set(osqp_RES_DIRS_RELEASE )
set(osqp_DEFINITIONS_RELEASE )
set(osqp_SHARED_LINK_FLAGS_RELEASE )
set(osqp_EXE_LINK_FLAGS_RELEASE )
set(osqp_OBJECTS_RELEASE )
set(osqp_COMPILE_DEFINITIONS_RELEASE )
set(osqp_COMPILE_OPTIONS_C_RELEASE )
set(osqp_COMPILE_OPTIONS_CXX_RELEASE )
set(osqp_LIB_DIRS_RELEASE "${osqp_PACKAGE_FOLDER_RELEASE}/lib")
set(osqp_LIBS_RELEASE osqp)
set(osqp_SYSTEM_LIBS_RELEASE m rt dl)
set(osqp_FRAMEWORK_DIRS_RELEASE )
set(osqp_FRAMEWORKS_RELEASE )
set(osqp_BUILD_DIRS_RELEASE "${osqp_PACKAGE_FOLDER_RELEASE}/")

# COMPOUND VARIABLES
set(osqp_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${osqp_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${osqp_COMPILE_OPTIONS_C_RELEASE}>")
set(osqp_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${osqp_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${osqp_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${osqp_EXE_LINK_FLAGS_RELEASE}>")


set(osqp_COMPONENTS_RELEASE )