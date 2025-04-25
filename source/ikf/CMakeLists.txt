
#
# External dependencies
#

# find_package(THIRDPARTY REQUIRED)


#
# Library name and options
#

# Target name
set(target ikf)

# Exit here if required dependencies are not met
message(STATUS "Lib ${target}")

# Set API export file and macro
string(MAKE_C_IDENTIFIER ${target} target_id)
string(TOUPPER ${target_id} target_id)
set(feature_file         "include/${target}/${target}_features.h")
set(export_file          "include/${target}/${target}_export.h")
set(template_export_file "include/${target}/${target}_api.h")
set(export_macro         "${target_id}_API")


#
# Sources
#

set(include_path "${CMAKE_CURRENT_SOURCE_DIR}/include/${target}")
set(include_local_path "${CMAKE_CURRENT_SOURCE_DIR}/include_local/${target}")
set(source_path  "${CMAKE_CURRENT_SOURCE_DIR}/source")

set(headers
 include/ikf/Logger/Logger.hpp

 include/ikf/Estimate/IBelief.hpp
 include/ikf/Estimate/LinearBelief.hpp
 include/ikf/EstimatorStd/KalmanFilterStd.hpp
 include/ikf/EstimatorStd/IsolatedKalmanFilterStd.hpp
 include/ikf/EstimatorStd/IKFHandlerStd.hpp

 #include/ikf/Estimator/IDiCoFilter.hpp
 include/ikf/Estimator/IIsolatedKalmanFilter.hpp
 include/ikf/Estimator/IIsolatedKalmanFilterCorr.hpp
 include/ikf/EstimatorHandler/IDICOHandler.hpp

include/ikf/EstimatorHandler/eFusionStrategy.hpp

 include/ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp
 include/ikf/EstimatorHandler/DecoupledPropagationHandler.hpp
 include/ikf/EstimatorHandler/ICSE_IKF_Handler.hpp
 include/ikf/EstimatorHandler/DCI_IKF_Handler.hpp
 include/ikf/EstimatorHandler/C_IKF_Handler.hpp
 include/ikf/EstimatorHandler/IMultiAgentHandler.hpp


 include/ikf/Container/Timestamp.hpp
 include/ikf/Container/TStampedData.hpp
 include/ikf/Container/THistoryBuffer.hpp
 include/ikf/utils/math.hpp
 include/ikf/utils/QuaternionUtils.hpp
 include/ikf/utils/IO.hpp
 include/ikf/utils/Rate.hpp
 include/ikf/utils/CyclicThread.hpp
 include/ikf/utils/ProgressBar.hpp
 include/ikf/utils/RandomSampler.hpp
 include/ikf/utils/GaussianNoiseGen.hpp
 include/ikf/utils/rand_op.hpp
 include/ikf/utils/MultivariateNormal.hpp
 include/ikf/utils/UnivariateNormal.hpp
 include/ikf/utils/lock_guard_timed.hpp

 ${include_local_path}/helper.hpp
)

set(sources
 source/Logger/Logger.cpp

 source/utils/eigen_utils.cpp
 source/utils/math.cpp
 source/utils/RandomSampler.cpp
 source/utils/GaussianNoiseGen.cpp
 source/utils/IO.cpp
 source/utils/Rate.cpp
 source/utils/CyclicThread.cpp
 source/utils/CSVTool.cpp
 source/utils/ProgressBar.cpp

 source/Containers/Timestamp.cpp
 source/Measurement/MeasData.cpp
 source/Estimate/IBelief.cpp
 source/Estimate/LinearBelief.cpp
 source/Estimator/ProcessMeasResult_t.cpp
 source/Estimator/KalmanFilter.cpp
 source/Estimator/NormalizedInnovationSquared.cpp
 source/Estimator/IKalmanFilter.cpp
 #source/Estimator/IDiCoFilter.cpp
 source/Estimator/IIsolatedKalmanFilter.cpp
 source/Estimator/IIsolatedKalmanFilterCorr.cpp

 source/EstimatorStd/KalmanFilterStd.cpp
 source/EstimatorStd/IsolatedKalmanFilterStd.cpp
 source/EstimatorStd/IKFHandlerStd.cpp

 #source/EstimatorHandler/IDiCoFilterHandler.cpp
 source/EstimatorHandler/eFusionStrategy.cpp

 source/EstimatorHandler/IDICOHandler.cpp
 source/EstimatorHandler/IsolatedKalmanFilterHandler.cpp
 source/EstimatorHandler/DecoupledPropagationHandler.cpp
 source/EstimatorHandler/ICSE_IKF_Handler.cpp
 source/EstimatorHandler/C_IKF_Handler.cpp
 source/EstimatorHandler/DCI_IKF_Handler.cpp
 source/EstimatorHandler/IMultiAgentHandler.cpp
)

# Group source files
set(header_group "Header Files (API)")
set(source_group "Source Files")
source_group_by_path(${include_path} "\\\\.h$|\\\\.hpp$"
    ${header_group} ${headers})
source_group_by_path(${source_path}  "\\\\.cpp$|\\\\.c$|\\\\.h$|\\\\.hpp$"
    ${source_group} ${sources})


#
# Create library
#

# Build library
add_library(${target}
    ${sources}
    ${headers}
)

# Create namespaced alias
add_library(${META_PROJECT_NAME}::${target} ALIAS ${target})

# Export library for downstream projects
export(TARGETS ${target} NAMESPACE ${META_PROJECT_NAME}:: FILE ${PROJECT_BINARY_DIR}/cmake/${target}/${target}-export.cmake)

# Create feature detection header
# Compilers: https://cmake.org/cmake/help/v3.1/variable/CMAKE_LANG_COMPILER_ID.html#variable:CMAKE_%3CLANG%3E_COMPILER_ID
# Feature: https://cmake.org/cmake/help/v3.1/prop_gbl/CMAKE_CXX_KNOWN_FEATURES.html
write_compiler_detection_header(
    FILE ${feature_file}
    PREFIX ${target_id}
    COMPILERS AppleClang Clang GNU MSVC
    FEATURES cxx_alignas cxx_alignof cxx_constexpr cxx_final cxx_noexcept cxx_nullptr cxx_sizeof_member cxx_thread_local
    VERSION 3.20
)

# Create API export header
generate_export_header(${target}
    EXPORT_FILE_NAME  ${export_file}
    EXPORT_MACRO_NAME ${export_macro}
)

generate_template_export_header(${target}
    ${target_id}
    ${template_export_file}
)


#
# Project options
#

set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
    INSTALL_RPATH "${LIBRARY_INSTALL_RPATH}"
    FOLDER "${IDE_FOLDER}"
    VERSION "${META_VERSION}"
    SOVERSION "${META_VERSION_MAJOR}"
)


#
# Include directories
#

target_include_directories(${target}
    PRIVATE
    ${PROJECT_BINARY_DIR}/source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include_local
    ${CMAKE_CURRENT_BINARY_DIR}/include

    #${matplot_SOURCE_DIR}/source
    PUBLIC
    ${DEFAULT_INCLUDE_DIRECTORIES}

    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


#
# Libraries
#

target_link_libraries(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_LIBRARIES}
    Eigen
    spdlog

    INTERFACE
    $<BUILD_INTERFACE:Eigen>
    $<BUILD_INTERFACE:spdlog>
)


#
# Compile definitions
#

target_compile_definitions(${target}
    PRIVATE

    PUBLIC
    $<$<NOT:$<BOOL:${BUILD_SHARED_LIBS}>>:${target_id}_STATIC_DEFINE>
    ${DEFAULT_COMPILE_DEFINITIONS}

    INTERFACE
)


#
# Compile options
#

target_compile_options(${target}
    PRIVATE
    ${DEFAULT_COMPILE_OPTIONS_PRIVATE}
    PUBLIC
    ${DEFAULT_COMPILE_OPTIONS_PUBLIC}

    INTERFACE
)


#
# Linker options
#

target_link_options(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_LINKER_OPTIONS}

    INTERFACE
)


#
# Target Health
#

perform_health_checks(
    ${target}
    ${sources}
    ${headers}
)


#
# Deployment
#

# Library
install(TARGETS ${target}
    EXPORT  "${target}-export"            COMPONENT dev
    RUNTIME DESTINATION ${INSTALL_BIN}    COMPONENT runtime
    LIBRARY DESTINATION ${INSTALL_SHARED} COMPONENT runtime
    ARCHIVE DESTINATION ${INSTALL_LIB}    COMPONENT dev
)

# Header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${target} DESTINATION ${INSTALL_INCLUDE}
    COMPONENT dev
)

# Generated header files
install(DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/include/${target} DESTINATION ${INSTALL_INCLUDE}
    COMPONENT dev
)

# CMake config
install(EXPORT ${target}-export
    NAMESPACE   ${META_PROJECT_NAME}::
    DESTINATION ${INSTALL_CMAKE}/${target}
    COMPONENT   dev
)
