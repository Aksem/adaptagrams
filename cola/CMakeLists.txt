cmake_minimum_required(VERSION 3.16)
project(adaptagrams)

# adaptagrams libs and GoogleTest require at least C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MFC_FLAG 2)
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
            URL https://github.com/google/googletest/archive/af29db7ec28d6df1c7f0f745186884091e602e07.zip
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
