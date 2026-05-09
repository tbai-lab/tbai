cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tbai_estim)

set(TBAI_ESTIM_SOURCES
    # InEKF
    src/inekf/InEKF.cpp
    src/inekf/LieGroup.cpp
    src/inekf/NoiseParams.cpp
    src/inekf/RobotState.cpp
    src/inekf/InEKFEstimator.cpp
)

if(TBAI_BUILD_VO)
    list(APPEND TBAI_ESTIM_SOURCES
        src/vision/PointCloudIndexer.cpp
        src/vision/SuperPointExtractor.cpp
        src/vision/VisualLandmarkTracker.cpp
    )
endif()

add_library(tbai_estim ${TBAI_ESTIM_SOURCES})
set_target_properties(tbai_estim PROPERTIES EXPORT_NAME estim)
target_link_libraries(tbai_estim PUBLIC tbai_core)
target_include_directories(tbai_estim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(TBAI_BUILD_VO)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d)
    target_link_libraries(tbai_estim
        PUBLIC
            opencv_core
            opencv_imgproc
            opencv_features2d
            onnxruntime
    )
    target_include_directories(tbai_estim
        PUBLIC
            $<BUILD_INTERFACE:${ONNXRUNTIME_INCLUDE_DIR}>
            $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>
    )
    target_compile_definitions(tbai_estim PUBLIC TBAI_ESTIM_HAS_VO=1)
endif()

if(TBAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()

## Installation rules
include(GNUInstallDirs)

install(TARGETS tbai_estim
    EXPORT tbai_estim-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT tbai_estim-targets
    NAMESPACE tbai::
    FILE tbai-estimator-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tbai
)
