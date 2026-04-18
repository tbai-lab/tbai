include(FetchContent)
include(GNUInstallDirs)

if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "[TBAI] ONNX Runtime fetch currently only supports Linux platforms")
endif()

if(NOT DEFINED TBAI_ONNXRUNTIME_VERSION OR "${TBAI_ONNXRUNTIME_VERSION}" STREQUAL "")
    set(TBAI_ONNXRUNTIME_VERSION "1.22.0")
endif()
set(ONNXRUNTIME_VERSION "${TBAI_ONNXRUNTIME_VERSION}")
set(ONNXRUNTIME_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz")

message(STATUS "[TBAI] Fetching ONNX Runtime v${ONNXRUNTIME_VERSION} from ${ONNXRUNTIME_URL}...")

FetchContent_Declare(
    onnxruntime
    URL ${ONNXRUNTIME_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(onnxruntime)

FetchContent_GetProperties(onnxruntime SOURCE_DIR ONNXRUNTIME_SOURCE_DIR)

set(ONNXRUNTIME_INCLUDE_DIR "${onnxruntime_SOURCE_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${onnxruntime_SOURCE_DIR}/lib")

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

message(STATUS "[TBAI] ONNX Runtime configured: ${onnxruntime_SOURCE_DIR}")

install(DIRECTORY "${ONNXRUNTIME_INCLUDE_DIR}/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.${ONNXRUNTIME_VERSION}"
              "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1"
              "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
