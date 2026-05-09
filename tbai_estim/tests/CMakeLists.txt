find_package(GTest REQUIRED)

function(add_tbai_estim_test TEST_NAME)
    add_executable(${TEST_NAME} ${ARGN})
    target_link_libraries(${TEST_NAME}
        PRIVATE
            tbai_estim
            tbai_core
            GTest::GTest
            GTest::Main
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

if(TBAI_BUILD_VO)
    add_tbai_estim_test(test_pointcloud_indexer src/test_pointcloud_indexer.cpp)
    add_tbai_estim_test(test_visual_landmark_correction src/test_visual_landmark_correction.cpp)
    add_tbai_estim_test(test_superpoint_extractor src/test_superpoint_extractor.cpp)
    add_tbai_estim_test(test_visual_landmark_tracker src/test_visual_landmark_tracker.cpp)
    add_tbai_estim_test(test_vo_end_to_end src/test_vo_end_to_end.cpp)
endif()
