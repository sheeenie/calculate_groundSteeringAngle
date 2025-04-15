cmake_minimum_required(VERSION 3.2)
project(helloworld)

set(CMAKE_CXX_STANDARD 11)

# Include headers from include/
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Build main app
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/helloworld.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PrimeChecker.cpp
)

# Enable testing
enable_testing()

# Build test runner
add_executable(${PROJECT_NAME}-Runner
    ${CMAKE_CURRENT_SOURCE_DIR}/test/TestPrimeChecker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PrimeChecker.cpp
)

# Register test
add_test(NAME ${PROJECT_NAME}-Runner COMMAND ${PROJECT_NAME}-Runner)
