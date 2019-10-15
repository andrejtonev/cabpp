cmake_minimum_required(VERSION 3.10)

project(cabpp VERSION 1.0.0)

find_package (Threads)

# Create the cabpp header-only library
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE ${CMAKE_THREAD_LIBS_INIT})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/CABppConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS ${PROJECT_NAME}
    EXPORT CABppTargets
    LIBRARY DESTINATION lib COMPONENT Runtime
    ARCHIVE DESTINATION lib COMPONENT Development
    RUNTIME DESTINATION bin COMPONENT Runtime
    PUBLIC_HEADER DESTINATION include COMPONENT Development
    BUNDLE DESTINATION bin COMPONENT Runtime
)

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/CABppConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/CABppConfig.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(EXPORT CABppTargets DESTINATION lib/cmake/${PROJECT_NAME})
install(FILES "${PROJECT_BINARY_DIR}/CABppConfigVersion.cmake"
              "${PROJECT_BINARY_DIR}/CABppConfig.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)

option(BUILD_EXAMPLE "Compile the example application" OFF)
if (BUILD_EXAMPLE)
  add_executable(${PROJECT_NAME}-${PROJECT_VERSION} main.cpp)
  target_include_directories(${PROJECT_NAME}-${PROJECT_VERSION} PRIVATE include)
  target_link_libraries(${PROJECT_NAME}-${PROJECT_VERSION} ${CMAKE_THREAD_LIBS_INIT}
                                                           cabpp)
  install(TARGETS ${PROJECT_NAME}-${PROJECT_VERSION} RUNTIME DESTINATION bin)
endif()

option(GTEST "Compile library gtests" OFF)
if (GTEST)
  enable_testing()
  add_subdirectory(tests)
endif()
