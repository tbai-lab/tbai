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
    src/robots/bindings_go2.cpp
    src/robots/bindings_go2_unitree.cpp
    src/robots/bindings_g1.cpp
    src/robots/bindings_g1_unitree.cpp
    src/robots/bindings_franka.cpp
    src/robots/bindings_anymal_b.cpp
    src/robots/bindings_anymal_c.cpp
    src/robots/bindings_anymal_d.cpp
    src/robots/bindings_spot.cpp
    src/controllers/bindings_static.cpp
    src/controllers/bindings_bob.cpp
    src/controllers/bindings_np3o.cpp
    src/controllers/bindings_wtw.cpp
    src/controllers/bindings_central.cpp
    src/controllers/bindings_arm_mpc.cpp
    src/controllers/bindings_quadruped_mpc.cpp
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

if(TBAI_BUILD_DEPLOY_GO2_UNITREE)
    target_link_libraries(_C PUBLIC tbai_deploy_go2_unitree)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_GO2_UNITREE)
endif()

if(TBAI_BUILD_DEPLOY_G1)
    target_link_libraries(_C PUBLIC tbai_deploy_g1)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_G1)
endif()

if(TBAI_BUILD_DEPLOY_G1_UNITREE)
    target_link_libraries(_C PUBLIC tbai_deploy_g1_unitree)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_G1_UNITREE)
endif()

if(TBAI_BUILD_DEPLOY_FRANKA)
    target_link_libraries(_C PUBLIC tbai_deploy_franka)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_FRANKA)
endif()

if(TBAI_BUILD_DEPLOY_ANYMAL_B)
    target_link_libraries(_C PUBLIC tbai_deploy_anymal_b)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_ANYMAL_B)
endif()

if(TBAI_BUILD_DEPLOY_ANYMAL_C)
    target_link_libraries(_C PUBLIC tbai_deploy_anymal_c)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_ANYMAL_C)
endif()

if(TBAI_BUILD_DEPLOY_ANYMAL_D)
    target_link_libraries(_C PUBLIC tbai_deploy_anymal_d)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_ANYMAL_D)
endif()

if(TBAI_BUILD_DEPLOY_SPOT)
    target_link_libraries(_C PUBLIC tbai_deploy_spot)
    target_compile_definitions(_C PRIVATE TBAI_HAS_DEPLOY_SPOT)
endif()

if(TBAI_BUILD_MPC)
    target_link_libraries(_C PUBLIC tbai_arm_mpc tbai_quadruped_mpc)
    target_compile_definitions(_C PRIVATE TBAI_HAS_ARM_MPC TBAI_HAS_QUADRUPED_MPC)
endif()
target_include_directories(_C
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the _C extension module into the tbai package
install(TARGETS _C LIBRARY DESTINATION tbai)