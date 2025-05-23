
# 
# External dependencies
# 

find_package(${META_PROJECT_NAME} REQUIRED HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../")

# 
# Executable name and options
#

# Target name
set(target ikf-test)
set(target_lib ikf)
message(STATUS "Test ${target}")


# 
# Sources
# 

set(sources
    main.cpp
    ikf_THistoryBuffer_test.cpp
    ikf_TMultiHistoryBuffer_test.cpp
    ikf_TTHimeHorizonBuffer.cpp
    ikf_IKalmanFilter_test.cpp
    #
    ikf_eigen_mvn_test.cpp
    ikf_Timestamp_test.cpp
    #ikf_LinearIKFStd_test.cpp
)


# 
# Create executable
# 

# Build executable
add_executable(${target}
    ${sources}
)

# Create namespaced alias
add_executable(${META_PROJECT_NAME}::${target} ALIAS ${target})


# 
# Project options
# 

set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
    FOLDER "${IDE_FOLDER}"
)


# 
# Include directories
# 

target_include_directories(${target}
    PRIVATE
    ${DEFAULT_INCLUDE_DIRECTORIES}
    ${PROJECT_BINARY_DIR}/source/include
)


# 
# Libraries
# 

target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LIBRARIES}
    ${META_PROJECT_NAME}::${target_lib}
    googletest::googletest
)


# 
# Compile definitions
# 

target_compile_definitions(${target}
    PRIVATE
    ${DEFAULT_COMPILE_DEFINITIONS}
)


# 
# Compile options
# 

target_compile_options(${target}
    PRIVATE
    ${DEFAULT_COMPILE_OPTIONS_PRIVATE}
    PUBLIC
    ${DEFAULT_COMPILE_OPTIONS_PUBLIC}
)


# 
# Linker options
# 

target_link_options(${target}
    PRIVATE
    ${DEFAULT_LINKER_OPTIONS}
)
