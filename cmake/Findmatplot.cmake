include(FetchContent)
include(ExternalProject)
include(CPM)

CPMAddPackage(
    NAME matplotplusplus
    GITHUB_REPOSITORY alandefreitas/matplotplusplus
    GIT_TAG origin/master # or whatever tag you want
    CMAKE_ARGS


)

# patch fix missing include file!
add_custom_target(copy-files-matplot ALL
   COMMAND ${CMAKE_COMMAND} -E copy_if_different
   ${CMAKE_BINARY_DIR}/_deps/matplotplusplus-build/source/matplot/matplot/detail/exports.h
   ${CMAKE_BINARY_DIR}/_deps/matplotplusplus-src/source/matplot/detail/exports.h
   )


# TODO: hack needed to link correctly! How can this be fixed?
add_library(matplot_imp SHARED IMPORTED)
add_dependencies(matplot_imp matplotplusplus copy-files-matplot)
#set(MATPLOT_INCLUDE_DIRECTORY "${CMAKE_BINARY_DIR}/matplot/include")
#set(MATPLOT_LIB_PATH "${CMAKE_BINARY_DIR}/matplot/lib/libmatplot.so")
list(APPEND MATPLOT_INCLUDE_DIR
  ${CMAKE_BINARY_DIR}/_deps/matplotplusplus-src/source
 )

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set_target_properties(matplot_imp PROPERTIES
    IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/libmatplotd.so
    INTERFACE_INCLUDE_DIRECTORIES "${MATPLOT_INCLUDE_DIR}"
  )
else()
 set_target_properties(matplot_imp PROPERTIES
   IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/libmatplot.so
   INTERFACE_INCLUDE_DIRECTORIES "${MATPLOT_INCLUDE_DIR}"
 )
endif()

