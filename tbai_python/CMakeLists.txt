cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_python)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)
find_package(pinocchio CONFIG REQUIRED)

nanobind_add_module(_C
    src/bindings.cpp
)
target_link_libraries(_C
    PUBLIC
        tbai_core
        tbai_bob
        tbai_np3o
        tbai_wtw
        tbai_reference
        tbai_static
        tbai_estim
        pinocchio::pinocchio
)

if(TBAI_BUILD_DEPLOY_GO2)
    target_link_libraries(_C PUBLIC tbai_deploy_go2)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_GO2)
endif()

if(TBAI_BUILD_DEPLOY_G1)
    target_link_libraries(_C PUBLIC tbai_deploy_g1)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_G1)
endif()
target_include_directories(_C
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the _C extension module into the tbai package
install(TARGETS _C LIBRARY DESTINATION tbai)