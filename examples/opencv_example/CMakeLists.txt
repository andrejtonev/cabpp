find_package(Threads)
find_package(OpenCV REQUIRED)

set(EXP_NAME opencv_example-${PROJECT_NAME})

add_executable(${EXP_NAME} opencv_example.cpp)
target_include_directories(${EXP_NAME} PRIVATE include)
target_link_libraries(${EXP_NAME} ${CMAKE_THREAD_LIBS_INIT}
                                  ${OpenCV_LIBS}
                                  cabpp
                                  )
                                  
install(TARGETS ${EXP_NAME} RUNTIME DESTINATION bin)
