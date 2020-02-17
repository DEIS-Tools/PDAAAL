cmake_minimum_required(VERSION 3.7)

cmake_policy(SET CMP0048 NEW)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(PDAAAL_BuildTests "Build the unit tests when BUILD_TESTING is enabled." ON)
option(PDAAAL_AddressSanitizer "Enables address sanitization during compilation." OFF)

project(pdaaal VERSION 1.0.0 LANGUAGES CXX)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wpedantic -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG -Wall -Wpedantic -fPIC")
set(CMAKE_CXX_STANDARD 17)

if(PDAAAL_AddressSanitizer)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -fno-omit-frame-pointer -fsanitize=address")
endif(PDAAAL_AddressSanitizer)

#actual library
add_subdirectory(src)

if(BUILD_TESTING AND PDAAAL_BuildTests)
    #benchmark
    add_subdirectory(benchmark)

    #testing
    add_subdirectory(test)
    enable_testing()
    add_test(NAME Test1     COMMAND Test1)
endif()