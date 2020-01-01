find_package(Threads)

set(EXP_NAME mutex_example-${PROJECT_NAME})

add_executable(${EXP_NAME} mutex_example.cpp)
target_include_directories(${EXP_NAME} PRIVATE include)
target_link_libraries(${EXP_NAME} ${CMAKE_THREAD_LIBS_INIT}
                                  cabpp
                                  )
                                  
install(TARGETS ${EXP_NAME} RUNTIME DESTINATION bin)
