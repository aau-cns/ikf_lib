
# Check if examples are enabled
if(NOT OPTION_BUILD_EXAMPLES)
    return()
endif()

# Example applications
add_subdirectory(ikf_simple_cmd)
add_subdirectory(ikf_delay_cmd)
