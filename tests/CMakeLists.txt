include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(googletest)

add_executable(spacecraft_tests
    test_placeholder.cpp
)

target_link_libraries(spacecraft_tests
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(spacecraft_tests)