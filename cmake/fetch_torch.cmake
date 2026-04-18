include(FetchContent)
include(GNUInstallDirs)

option(TBAI_FORCE_TORCH_FETCH "Force fetching libtorch from PyTorch website" OFF)


if(TBAI_FORCE_TORCH_FETCH)
    message(STATUS "[TBAI] Force fetching libtorch from PyTorch website.")
endif()

find_package(Python3 COMPONENTS Interpreter)

if(NOT Python3_Interpreter_FOUND)
    message(FATAL_ERROR "[TBAI] Python3 interpreter not found. Please install Python3.")
endif()

set(TORCH_FROM_PYTHON FALSE)

if(NOT TBAI_FORCE_TORCH_FETCH)
    # Try to import torch from the Python interpreter and verify its ABI matches
    # what setup.py enforces (CXX11 ABI == 1). Mismatched ABI causes symbol
    # conflicts, so we fall back to fetching a known-good libtorch in that case.
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import os, torch; print(os.path.dirname(torch.__file__)); print(int(torch.compiled_with_cxx11_abi()))"
        RESULT_VARIABLE PYTHON_TORCH_RESULT
        OUTPUT_VARIABLE PYTHON_TORCH_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    if(PYTHON_TORCH_RESULT EQUAL 0)
        string(REPLACE "\n" ";" PYTHON_TORCH_LINES "${PYTHON_TORCH_OUTPUT}")
        list(GET PYTHON_TORCH_LINES 0 TORCH_PYTHON_PACKAGE_DIR)
        list(GET PYTHON_TORCH_LINES 1 TORCH_PYTHON_ABI)

        if(TORCH_PYTHON_ABI EQUAL 1)
            message(STATUS "[TBAI] PyTorch found via Python interpreter (CXX11 ABI), skipping fetch.")
            set(TORCH_SOURCE_DIR "${TORCH_PYTHON_PACKAGE_DIR}")
            set(TORCH_FROM_PYTHON TRUE)
        else()
            message(STATUS "[TBAI] PyTorch found via Python but ABI is ${TORCH_PYTHON_ABI} (expected 1 / CXX11). Falling back to fetch.")
        endif()
    endif()
endif()

if(NOT TORCH_FROM_PYTHON)
    # Download and configure libtorch CPU
    if(NOT DEFINED TBAI_TORCH_VERSION OR "${TBAI_TORCH_VERSION}" STREQUAL "")
        set(TBAI_TORCH_VERSION "2.7.0")
    endif()
    set(PYTORCH_VERSION "${TBAI_TORCH_VERSION}")
    set(PYTORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip")
    message(STATUS "[TBAI] PyTorch not found via Python interpreter, fetching libtorch version ${PYTORCH_VERSION} from ${PYTORCH_URL}.")

    FetchContent_Declare(
        libtorch
        URL ${PYTORCH_URL}
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

    FetchContent_MakeAvailable(libtorch)

    # Add libtorch to path
    set(TORCH_SOURCE_DIR "${libtorch_SOURCE_DIR}")
    list(APPEND CMAKE_PREFIX_PATH ${libtorch_SOURCE_DIR})
endif()

if(TORCH_FROM_PYTHON)
    # Bypass the real TorchConfig.cmake. When the installed torch wheel is a CUDA
    # build (the pip default), its Caffe2Config.cmake hard-fails if the host has
    # no CUDA toolkit. We only need CPU-side libs — write a shim TorchConfig that
    # exposes libtorch.so / libtorch_cpu.so / libc10.so from the installed wheel.
    # Every find_package(Torch REQUIRED) across the repo resolves to this shim.
    set(_tbai_torch_shim_dir "${CMAKE_BINARY_DIR}/tbai_torch_shim")
    file(MAKE_DIRECTORY "${_tbai_torch_shim_dir}")
    set(_tbai_torch_shim_template [=[
set(TORCH_INSTALL_PREFIX "@TORCH_SOURCE_DIR@")
set(TORCH_INCLUDE_DIRS
    "@TORCH_SOURCE_DIR@/include"
    "@TORCH_SOURCE_DIR@/include/torch/csrc/api/include"
)
set(TORCH_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=1")

if(NOT TARGET c10)
    add_library(c10 SHARED IMPORTED)
    set_target_properties(c10 PROPERTIES
        IMPORTED_LOCATION "@TORCH_SOURCE_DIR@/lib/libc10.so"
    )
endif()
if(NOT TARGET torch_cpu)
    add_library(torch_cpu SHARED IMPORTED)
    set_target_properties(torch_cpu PROPERTIES
        IMPORTED_LOCATION "@TORCH_SOURCE_DIR@/lib/libtorch_cpu.so"
        INTERFACE_LINK_LIBRARIES "c10"
    )
endif()
if(NOT TARGET torch)
    add_library(torch SHARED IMPORTED)
    set_target_properties(torch PROPERTIES
        IMPORTED_LOCATION "@TORCH_SOURCE_DIR@/lib/libtorch.so"
        INTERFACE_INCLUDE_DIRECTORIES "@TORCH_SOURCE_DIR@/include;@TORCH_SOURCE_DIR@/include/torch/csrc/api/include"
        INTERFACE_COMPILE_OPTIONS "-D_GLIBCXX_USE_CXX11_ABI=1"
        INTERFACE_LINK_LIBRARIES "torch_cpu;c10"
    )
endif()

set(TORCH_LIBRARIES torch)
set(Torch_FOUND TRUE)
]=])
    string(CONFIGURE "${_tbai_torch_shim_template}" _tbai_torch_shim_content @ONLY)
    file(WRITE "${_tbai_torch_shim_dir}/TorchConfig.cmake" "${_tbai_torch_shim_content}")
    set(Torch_DIR "${_tbai_torch_shim_dir}" CACHE PATH "tbai torch shim (CPU-only bypass of CUDA-torch TorchConfig)" FORCE)
endif()

find_package(Torch REQUIRED)

macro(_tbai_print_var label var)
    if(DEFINED ${var} AND NOT "${${var}}" STREQUAL "")
        message(STATUS "[TBAI] ${label}: ${${var}}")
    else()
        message(STATUS "[TBAI] ${label}: <not set>")
    endif()
endmacro()

_tbai_print_var("Torch_DIR found" Torch_DIR)
_tbai_print_var("Torch libraries" TORCH_LIBRARIES)
_tbai_print_var("Torch include dir" TORCH_INCLUDE_DIRS)
_tbai_print_var("Torch library dir" TORCH_LIBRARY_DIRS)
_tbai_print_var("Torch source dir" TORCH_SOURCE_DIR)
_tbai_print_var("Torch libraries" Torch_LIBRARIES)
_tbai_print_var("Torch version" Torch_VERSION)
_tbai_print_var("Torch config" Torch_CONFIG)
_tbai_print_var("Torch module" Torch_MODULE_PATH)
_tbai_print_var("Torch package" Torch_PACKAGE_DIR)
_tbai_print_var("Torch package version" Torch_PACKAGE_VERSION)
_tbai_print_var("Torch package include dir" Torch_PACKAGE_INCLUDE_DIRS)

if(NOT DEFINED TORCH_SOURCE_DIR OR "${TORCH_SOURCE_DIR}" STREQUAL "")
    message(FATAL_ERROR "TORCH_SOURCE_DIR is not set or is empty. PyTorch/LibTorch was not found or configured properly.")
endif()


if(NOT TORCH_FROM_PYTHON)
    install(DIRECTORY ${TORCH_SOURCE_DIR}/lib/
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
            FILES_MATCHING PATTERN "*.so*"
        )

    install(DIRECTORY ${TORCH_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(DIRECTORY ${TORCH_SOURCE_DIR}/share/cmake/
        DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake
    )
endif()