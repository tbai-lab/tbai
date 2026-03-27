cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_wtw)

find_package(Torch REQUIRED)
find_package(pinocchio REQUIRED)

add_library(tbai_wtw
    src/WtwController.cpp
    src/HistoryBuffer.cpp
)
set_target_properties(tbai_wtw PROPERTIES EXPORT_NAME wtw)

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()

target_include_directories(tbai_wtw
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tbai_wtw
    PUBLIC
        tbai_core
        tbai_torch
        tbai_reference
        ${TORCH_LIBRARIES}
        pinocchio::pinocchio
)

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_wtw
    EXPORT tbai_wtw-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_wtw-targets
    NAMESPACE tbai::
    FILE tbai-wtw-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)
