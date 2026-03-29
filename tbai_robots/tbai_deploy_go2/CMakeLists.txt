cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_go2)

find_package(tbai_sdk REQUIRED)

add_library(tbai_deploy_go2
    src/Go2RobotInterface.cpp
)
set_target_properties(tbai_deploy_go2 PROPERTIES EXPORT_NAME deploy_go2)

target_include_directories(tbai_deploy_go2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(tbai_deploy_go2
    PUBLIC
        tbai_core
        tbai_estim
        tbai_sdk::tbai_sdk
        pthread
)

if(TBAI_BUILD_DEPLOY_GO2_UNITREE)
    add_library(tbai_deploy_go2_unitree
        src/Go2RobotInterfaceUnitree.cpp
    )
    set_target_properties(tbai_deploy_go2_unitree PROPERTIES EXPORT_NAME deploy_go2_unitree)

    target_include_directories(tbai_deploy_go2_unitree
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(tbai_deploy_go2_unitree
        PUBLIC
            tbai_core
            tbai_estim
            unitree_sdk2
    )
endif()


## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_deploy_go2
    EXPORT tbai_deploy_go2-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_deploy_go2-targets
    NAMESPACE tbai::
    FILE tbai-deploy_go2-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)

if(TBAI_BUILD_DEPLOY_GO2_UNITREE)
    install(TARGETS tbai_deploy_go2_unitree
        EXPORT tbai_deploy_go2_unitree-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(EXPORT tbai_deploy_go2_unitree-targets
        NAMESPACE tbai::
        FILE tbai-deploy_go2_unitree-targets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
    )
endif()
