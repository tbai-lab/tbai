cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_deploy_anymal_b)

set(TBAI_SDK_DIR "/home/kb/projects/tbai_sdk" CACHE PATH "Path to tbai_sdk")
set(ZENOH_INSTALL "${TBAI_SDK_DIR}/deps/zenoh-install" CACHE PATH "Zenoh install prefix")

add_library(tbai_deploy_anymal_b src/AnymalBRobotInterface.cpp)
set_target_properties(tbai_deploy_anymal_b PROPERTIES EXPORT_NAME deploy_anymal_b)

target_include_directories(tbai_deploy_anymal_b PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TBAI_SDK_DIR}/include
    ${ZENOH_INSTALL}/include
)
target_compile_definitions(tbai_deploy_anymal_b PUBLIC ZENOHCXX_ZENOHC)
target_link_directories(tbai_deploy_anymal_b PUBLIC ${ZENOH_INSTALL}/lib)
target_link_libraries(tbai_deploy_anymal_b PUBLIC tbai_core tbai_estim zenohc pthread)

include(GNUInstallDirs)
install(TARGETS tbai_deploy_anymal_b EXPORT tbai_deploy_anymal_b-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT tbai_deploy_anymal_b-targets NAMESPACE tbai:: FILE tbai-deploy_anymal_b-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai)
