if (NOT TARGET matplot)
 include(ExternalProject)
 ExternalProject_Add(matplot-ext
   PREFIX ${CMAKE_BINARY_DIR}/matplot
   GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
   GIT_TAG origin/master
   UPDATE_COMMAND ""
   #PATCH_COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/extra_version.patch
   CMAKE_ARGS
   -DBUILD_SHARED_LIBS=ON
   -DMATPLOTPP_BUILD_INSTALLER=ON
   -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/matplot
   -DCMAKE_BUILD_TYPE=Release
   -DMATPLOTPP_BUILD_EXAMPLES=OFF
   -DMATPLOTPP_BUILD_TESTS=OFF
   -DMATPLOTPP_BUILD_SHARED_LIBS=ON
   -DMATPLOTPP_BUILD_WITH_SANITIZERS=OFF
 )
 add_library(matplot SHARED IMPORTED GLOBAL)
 add_dependencies(matplot matplot-ext)
 file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/matplot/include)
 set_property(TARGET matplot PROPERTY INTERFACE_INCLUDE_DIRECTORIES
     ${CMAKE_BINARY_DIR}/matplot/include
     )

   set_target_properties(matplot PROPERTIES
     IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/matplot/lib/libmatplot.so)

endif()


