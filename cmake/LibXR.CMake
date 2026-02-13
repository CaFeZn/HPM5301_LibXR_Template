set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Path to your local LibXR repository.
set(LIBXR_DIR "D:/Codes/libxr" CACHE PATH "Path to LibXR source tree")

if(NOT EXISTS "${LIBXR_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "LibXR not found. Please set LIBXR_DIR correctly: ${LIBXR_DIR}")
endif()

# LibXR platform/driver selection
set(LIBXR_SYSTEM None CACHE STRING "LibXR system backend")
set(LIBXR_DRIVER hpm CACHE STRING "LibXR driver backend")
set(LIBXR_NO_EIGEN True CACHE BOOL "Disable Eigen for embedded targets")

# Optional XRobot modules
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Modules")
    set(XROBOT_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules")
endif()

# Import LibXR as a subproject
add_subdirectory("${LIBXR_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/libxr")

# Make LibXR compile with the same HPM SDK compile options/includes.
if(DEFINED HPM_SDK_LIB_ITF AND TARGET ${HPM_SDK_LIB_ITF})
    target_link_libraries(xr PUBLIC ${HPM_SDK_LIB_ITF})
endif()

# Let app sources directly include LibXR headers.
if(TARGET app)
    target_link_libraries(app PUBLIC xr)
endif()

# Ensure LibXR object files are linked into the final ELF target.
if(DEFINED APP_ELF_NAME AND TARGET ${APP_ELF_NAME})
    target_link_libraries(${APP_ELF_NAME} xr)
endif()

# Keep app in debug mode while shrinking LibXR binary size.
if(TARGET xr)
    string(TOLOWER "${CMAKE_BUILD_TYPE}" _libxr_build_type)
    if(_libxr_build_type STREQUAL "debug")
        target_compile_options(xr PRIVATE -O2)
    endif()
endif()
