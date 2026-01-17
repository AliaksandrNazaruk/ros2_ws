#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "nav2_util::nav2_util_core" for configuration "Release"
set_property(TARGET nav2_util::nav2_util_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nav2_util::nav2_util_core PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "bond::bond__rosidl_generator_c;bond::bond__rosidl_typesupport_fastrtps_c;bond::bond__rosidl_typesupport_fastrtps_cpp;bond::bond__rosidl_typesupport_introspection_c;bond::bond__rosidl_typesupport_c;bond::bond__rosidl_typesupport_introspection_cpp;bond::bond__rosidl_typesupport_cpp;bond::bond__rosidl_generator_py"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libnav2_util_core.so"
  IMPORTED_SONAME_RELEASE "libnav2_util_core.so"
  )

list(APPEND _cmake_import_check_targets nav2_util::nav2_util_core )
list(APPEND _cmake_import_check_files_for_nav2_util::nav2_util_core "${_IMPORT_PREFIX}/lib/libnav2_util_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
