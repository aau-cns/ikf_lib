
# Check if examples are enabled
if(NOT OPTION_BUILD_EXAMPLES)
    return()
endif()
if(NOT TARGET matplot)
 find_package(matplot REQUIRED)
endif()



message(" **************************************")
message(" --- Examples' Main Dependencies:")
message(" * matplot: ${MATPLOT_INCLUDE_DIR}, ${MATPLOT_VERSION} ")


message(" **************************************")

# Example applications
add_subdirectory(ikf_simple_cmd)
add_subdirectory(ikf_delay_cmd)
