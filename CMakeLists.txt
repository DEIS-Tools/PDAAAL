cmake_minimum_required(VERSION 3.14)

cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0069 NEW)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PDAAAL_VERSION 1.0.0)
project(pdaaal VERSION ${PDAAAL_VERSION} LANGUAGES CXX)


option(PDAAAL_BuildTests "Build the unit tests when BUILD_TESTING is enabled." ON)
option(PDAAAL_Build_Main "Build the stand-alone PDAAAL binary." ON)
option(PDAAAL_AddressSanitizer "Enables address sanitization during compilation." OFF)
option(PDAAAL_GetDependencies "Fetch external dependencies from web." ON)


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall -Wpedantic")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lstdc++")
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(PDAAAL_AddressSanitizer)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -fno-omit-frame-pointer -fsanitize=address")
endif(PDAAAL_AddressSanitizer)

find_package(Boost 1.70 CONFIG REQUIRED COMPONENTS headers program_options unit_test_framework)

#actual library
add_subdirectory(src)

set(BUILD_TESTING ON)

if(BUILD_TESTING AND PDAAAL_BuildTests)
    #benchmark
    add_subdirectory(benchmark)

    #testing
    add_subdirectory(test)
endif()
