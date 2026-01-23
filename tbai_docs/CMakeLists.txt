cmake_minimum_required(VERSION 3.16.0)
project(tbai_docs VERSION 2.0.0 LANGUAGES CXX)

option(TBAI_DOCS_BUILD_DOXYGEN "Build Doxygen API documentation" ON)
option(TBAI_DOCS_BUILD_SPHINX "Build Sphinx documentation" OFF)

if(TBAI_DOCS_BUILD_DOXYGEN)
    find_package(Doxygen REQUIRED)
endif()

if(TBAI_DOCS_BUILD_SPHINX)
    find_program(SPHINX_EXECUTABLE sphinx-build REQUIRED)
endif()

set(TBAI_DOCS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(TBAI_DOCS_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/docs)
set(TBAI_DOXYGEN_OUTPUT_DIR ${TBAI_DOCS_BUILD_DIR}/doxygen)
set(TBAI_SPHINX_OUTPUT_DIR ${TBAI_DOCS_BUILD_DIR}/html)

set(TBAI_HEADER_DIRS "")
set(TBAI_PACKAGES
    tbai_core
    tbai_torch
    tbai_reference
    tbai_static
    tbai_bob
    tbai_wtw
    tbai_np3o
    tbai_estim
    tbai_mpc
    tbai_dtc
    tbai_joe
    tbai_ocs2
    tbai_mujoco
    tbai_python
    tbai_deploy_go2
    tbai_deploy_go2w
    tbai_deploy_g1
)

foreach(pkg ${TBAI_PACKAGES})
    set(pkg_include_dir "${CMAKE_CURRENT_SOURCE_DIR}/../${pkg}/include")
    if(EXISTS ${pkg_include_dir})
        list(APPEND TBAI_HEADER_DIRS ${pkg_include_dir})
    endif()
endforeach()

string(REPLACE ";" " \\\n                         " DOXYGEN_INPUT_DIRS "${TBAI_HEADER_DIRS}")

if(TBAI_DOCS_BUILD_DOXYGEN)
    file(MAKE_DIRECTORY ${TBAI_DOXYGEN_OUTPUT_DIR})

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
        ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        @ONLY
    )

    add_custom_target(tbai_doxygen
        COMMAND ${CMAKE_COMMAND} -E make_directory ${TBAI_DOXYGEN_OUTPUT_DIR}
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()

if(TBAI_DOCS_BUILD_SPHINX)
    file(MAKE_DIRECTORY ${TBAI_SPHINX_OUTPUT_DIR})

    add_custom_target(tbai_sphinx_copy
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/source
            ${TBAI_DOCS_BUILD_DIR}/source
        COMMENT "Copying Sphinx source files"
    )

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/source/conf.py.in
        ${TBAI_DOCS_BUILD_DIR}/source/conf.py
        @ONLY
    )

    add_custom_target(tbai_sphinx
        COMMAND ${SPHINX_EXECUTABLE}
            -b html
            -c ${TBAI_DOCS_BUILD_DIR}/source
            ${TBAI_DOCS_BUILD_DIR}/source
            ${TBAI_SPHINX_OUTPUT_DIR}
        DEPENDS tbai_sphinx_copy
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Building documentation with Sphinx"
        VERBATIM
    )
endif()

if(TBAI_DOCS_BUILD_DOXYGEN AND TBAI_DOCS_BUILD_SPHINX)
    add_custom_target(tbai_docs ALL DEPENDS tbai_doxygen tbai_sphinx)
elseif(TBAI_DOCS_BUILD_DOXYGEN)
    add_custom_target(tbai_docs ALL DEPENDS tbai_doxygen)
elseif(TBAI_DOCS_BUILD_SPHINX)
    add_custom_target(tbai_docs ALL DEPENDS tbai_sphinx)
endif()

message(STATUS "tbai_docs: Doxygen=${TBAI_DOCS_BUILD_DOXYGEN}, Sphinx=${TBAI_DOCS_BUILD_SPHINX}")
