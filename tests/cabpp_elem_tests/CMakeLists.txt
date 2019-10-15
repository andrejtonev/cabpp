add_executable(cabpp_elem_tests ${TESTS_MAIN_PATH}
                                cabpp_elem_tests.cpp)

target_link_libraries(cabpp_elem_tests ${GTEST_BOTH_LIBRARIES} 
                                      ${CMAKE_THREAD_LIBS_INIT}
                                      cabpp)

gtest_discover_tests(cabpp_elem_tests)
                                      
install(TARGETS cabpp_elem_tests RUNTIME DESTINATION bin/tests)
