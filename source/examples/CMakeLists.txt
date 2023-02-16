
# Check if examples are enabled
if(NOT OPTION_BUILD_EXAMPLES)
    return()
endif()

find_package(matplot REQUIRED)

message(" **************************************")
message(" --- Main Dependencies:")
message(" * Eigen: ${EIGEN3_INCLUDE_DIR}, ${Eigen_VERSION} ")
message(" * matplot: ${MATPLOT_INCLUDE_DIR}, ${MATPLOT_VERSION} ")
message(" **************************************")

# Example applications
add_subdirectory(ikf_simple_cmd)
add_subdirectory(ikf_delay_cmd)
