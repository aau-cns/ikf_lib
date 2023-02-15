include(FetchContent)
include(ExternalProject)
include(CPM)

CPMAddPackage(
    NAME matplot
    GITHUB_REPOSITORY alandefreitas/matplotplusplus
    GIT_TAG origin/master # or whatever tag you want
)


add_library(matplot_imp SHARED IMPORTED)
add_dependencies(matplot_imp matplot)
#set(MATPLOT_INCLUDE_DIRECTORY "${CMAKE_BINARY_DIR}/matplot/include")
#set(MATPLOT_LIB_PATH "${CMAKE_BINARY_DIR}/matplot/lib/libmatplot.so")
set(MATPLOT_INCLUDE_DIR ${CMAKE_BINARY_DIR}/matplot/include)

set_target_properties(matplot_imp PROPERTIES
  IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/libmatplotd.so
  INTERFACE_INCLUDE_DIRECTORIES ${matplot_SOURCE_DIR}/source
)




get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()