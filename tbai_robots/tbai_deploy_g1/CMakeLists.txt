cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_g1)

find_package(tbai_sdk REQUIRED)

# Find Torch for G1Twist2Controller
find_package(Torch REQUIRED)

add_library(tbai_deploy_g1
    src/G1RobotInterface.cpp
    src/G1RLController.cpp
    src/G1MimicController.cpp
    src/G1BeyondMimicController.cpp
    src/G1SpinkickController.cpp
    src/G1Twist2Controller.cpp
    src/G1PBHCController.cpp
    src/G1ASAPController.cpp
    src/G1ASAPMimicController.cpp
    src/MotionLoader.cpp
    src/Twist2MotionLoader.cpp
    src/PBHCMotionLoader.cpp
)
set_target_properties(tbai_deploy_g1 PROPERTIES EXPORT_NAME deploy_g1)

target_include_directories(tbai_deploy_g1
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${ONNXRUNTIME_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
)


target_include_directories(tbai_deploy_g1
    PUBLIC


)

target_link_libraries(tbai_deploy_g1
    PUBLIC
        tbai_core
        tbai_reference
        tbai_estim
        tbai_torch
        tbai_sdk::tbai_sdk
        pthread
        onnxruntime
        ${TORCH_LIBRARIES}
)

if(TBAI_BUILD_DEPLOY_G1_UNITREE)
    add_library(tbai_deploy_g1_unitree
        src/G1RobotInterfaceUnitree.cpp
    )
    set_target_properties(tbai_deploy_g1_unitree PROPERTIES EXPORT_NAME deploy_g1_unitree)

    target_include_directories(tbai_deploy_g1_unitree
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(tbai_deploy_g1_unitree
        PUBLIC
            tbai_core
            tbai_estim
            unitree_sdk2
    )
endif()

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_deploy_g1
    EXPORT tbai_deploy_g1-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_deploy_g1-targets
    NAMESPACE tbai::
    FILE tbai-deploy_g1-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)

if(TBAI_BUILD_DEPLOY_G1_UNITREE)
    install(TARGETS tbai_deploy_g1_unitree
        EXPORT tbai_deploy_g1_unitree-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(EXPORT tbai_deploy_g1_unitree-targets
        NAMESPACE tbai::
        FILE tbai-deploy_g1_unitree-targets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
    )
endif()
