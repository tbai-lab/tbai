cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_anymal_d)

find_package(tbai_sdk REQUIRED)


add_library(tbai_deploy_anymal_d src/AnymalDRobotInterface.cpp)
set_target_properties(tbai_deploy_anymal_d PROPERTIES EXPORT_NAME deploy_anymal_d)

target_include_directories(tbai_deploy_anymal_d PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(tbai_deploy_anymal_d PUBLIC tbai_core tbai_estim tbai_sdk::tbai_sdk pthread)

include(GNUInstallDirs)
install(TARGETS tbai_deploy_anymal_d EXPORT tbai_deploy_anymal_d-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT tbai_deploy_anymal_d-targets NAMESPACE tbai:: FILE tbai-deploy_anymal_d-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai)
