macro(package_add_test TESTNAME)
    set(options)
    set(oneValueArgs SOURCE)
    set(multiValueArgs LIBRARIES DEFINITIONS)
    cmake_parse_arguments(PACKAGE_ADD_TEST "${options}" "${oneValueArgs}"
            "${multiValueArgs}" ${ARGN} )

    # create an executable in which the tests will be stored
    add_executable(${TESTNAME} ${PACKAGE_ADD_TEST_SOURCE})
    # link the Google test infrastructure, mocking library, and a default main function to
    # the test executable.  Remove g_test_main if writing your own main function.
    target_link_libraries(${TESTNAME} gtest gmock gtest_main ${PACKAGE_ADD_TEST_LIBRARIES})
    # gtest_discover_tests replaces gtest_add_tests,
    # see https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options to pass to it
    gtest_discover_tests(${TESTNAME}
            # set a working directory so your project root so that you can find test data via paths relative to the project root
            WORKING_DIRECTORY ${PROJECT_DIR}
            PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}"
    )
#    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
    target_compile_definitions(${TESTNAME} PRIVATE ${PACKAGE_ADD_TEST_DEFINITIONS})
endmacro()
