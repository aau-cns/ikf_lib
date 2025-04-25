
# 
# Configuration for all sub-projects
# 

# Generate version-header
configure_file(version.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/${META_PROJECT_NAME}/${META_PROJECT_NAME}-version.h)

#
# Configuration of basic libraries
#
#find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
#find_package(matplotplusplus REQUIRED)

# 
# Sub-projects
# 

#set(INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_BINARY_DIR}/include/")
# Libraries
set(IDE_FOLDER "")

add_subdirectory(ikf)

# Examples
set(IDE_FOLDER "Examples")
add_subdirectory(examples)

# Tests
if(OPTION_BUILD_TESTS)
    set(IDE_FOLDER "Tests")
    add_subdirectory(tests)
endif()


# 
# Deployment
# 

# Deploy generated headers
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/${META_PROJECT_NAME} DESTINATION include COMPONENT dev)
