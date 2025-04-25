
#
# Find doxygen
#

find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
    message(STATUS "Disabled generation of doxygen documentation (missing doxygen).")
    return()
endif()


#
# Target name
#

set(target api-docs)
message(STATUS "Doc ${target}")


#
# Configure Doxygen
#

set(DOXYGEN_QUIET                   YES)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE  ${PROJECT_SOURCE_DIR}/README.md)
set(DOXYGEN_PROJECT_LOGO            ${PROJECT_SOURCE_DIR}/resources/diagrams/ikf-lib-diag-Logo-IKF.png) # TODO replace by ${META_PROJECT_NAME}-logo.png
set(DOXYGEN_USE_MATHJAX             YES)
set(DOXYGEN_EXTRACT_ALL             YES)
set(DOXYGEN_EXTRACT_LOCAL_CLASSES   NO)


#
# Create Target
#

doxygen_add_docs(${target}
    ${PROJECT_SOURCE_DIR}/README.md
# library headers
    ${PROJECT_SOURCE_DIR}/source/ikf/include
    ${PROJECT_SOURCE_DIR}/source/ikf/include_local
# generated headers
    ${CMAKE_BINARY_DIR}/source/include
    ${CMAKE_BINARY_DIR}/source/ikf/include
    ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
add_dependencies(docs ${target})


#
# Deployment
#

install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION ${INSTALL_DOC}
    COMPONENT docs
)
