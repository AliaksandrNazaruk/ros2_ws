#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "nav2_route::route_server_core" for configuration "Release"
set_property(TARGET nav2_route::route_server_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nav2_route::route_server_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libroute_server_core.so"
  IMPORTED_SONAME_RELEASE "libroute_server_core.so"
  )

list(APPEND _cmake_import_check_targets nav2_route::route_server_core )
list(APPEND _cmake_import_check_files_for_nav2_route::route_server_core "${_IMPORT_PREFIX}/lib/libroute_server_core.so" )

# Import target "nav2_route::edge_scorers" for configuration "Release"
set_property(TARGET nav2_route::edge_scorers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nav2_route::edge_scorers PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libedge_scorers.so"
  IMPORTED_SONAME_RELEASE "libedge_scorers.so"
  )

list(APPEND _cmake_import_check_targets nav2_route::edge_scorers )
list(APPEND _cmake_import_check_files_for_nav2_route::edge_scorers "${_IMPORT_PREFIX}/lib/libedge_scorers.so" )

# Import target "nav2_route::route_operations" for configuration "Release"
set_property(TARGET nav2_route::route_operations APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nav2_route::route_operations PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libroute_operations.so"
  IMPORTED_SONAME_RELEASE "libroute_operations.so"
  )

list(APPEND _cmake_import_check_targets nav2_route::route_operations )
list(APPEND _cmake_import_check_files_for_nav2_route::route_operations "${_IMPORT_PREFIX}/lib/libroute_operations.so" )

# Import target "nav2_route::graph_file_loaders" for configuration "Release"
set_property(TARGET nav2_route::graph_file_loaders APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nav2_route::graph_file_loaders PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgraph_file_loaders.so"
  IMPORTED_SONAME_RELEASE "libgraph_file_loaders.so"
  )

list(APPEND _cmake_import_check_targets nav2_route::graph_file_loaders )
list(APPEND _cmake_import_check_files_for_nav2_route::graph_file_loaders "${_IMPORT_PREFIX}/lib/libgraph_file_loaders.so" )

# Import target "nav2_route::graph_file_savers" for configuration "Release"
set_property(TARGET nav2_route::graph_file_savers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nav2_route::graph_file_savers PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgraph_file_savers.so"
  IMPORTED_SONAME_RELEASE "libgraph_file_savers.so"
  )

list(APPEND _cmake_import_check_targets nav2_route::graph_file_savers )
list(APPEND _cmake_import_check_files_for_nav2_route::graph_file_savers "${_IMPORT_PREFIX}/lib/libgraph_file_savers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
