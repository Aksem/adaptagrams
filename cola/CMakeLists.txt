cmake_minimum_required(VERSION 3.16)
project(adaptagrams)

# adaptagrams libs require at least C++11, but GoogleTest at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# msvc requires AFXDLL definition to be able to use afx.h
# _USE_MATH_DEFINES is required for msvc to make M_PI and other constants available
add_definitions(-D_AFXDLL -D_USE_MATH_DEFINES)
set(CMAKE_MFC_FLAG 1)
set(VERSION 1.0.0)
set(SOVERSION 1)

if (BUILD_TESTING AND PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # testing setup was inspired by:
    # https://cliutils.gitlab.io/modern-cmake/chapters/testing.html
    # https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html

    # ctest requires explicit enabling testing even if BUILD_TESTING is provided
    enable_testing()
    include(FetchContent)
    # use new policy to add timestamps to fetched data
    cmake_policy(SET CMP0135 NEW)
    FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/b3a9ba2b8e975550799838332803d468797ae2e1.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    include(GoogleTest)
    include(cmake/Testing.cmake)
endif()

add_subdirectory(libavoid)
add_subdirectory(libcola)
add_subdirectory(libtopology)
add_subdirectory(libvpsc)
add_subdirectory(libdialect)
