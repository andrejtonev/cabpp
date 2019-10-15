find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

get_filename_component(TESTS_MAIN_PATH ./tests_main.cpp ABSOLUTE)

add_subdirectory(cabpp_elem_tests)
