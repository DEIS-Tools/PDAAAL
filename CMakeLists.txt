cmake_minimum_required(VERSION 3.7)

cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0069 NEW)

set(CMAKE_CXX_STANDARD 17)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
project(pdaaal VERSION 1.0.0 LANGUAGES CXX)


option(PDAAAL_BuildTests "Build the unit tests when BUILD_TESTING is enabled." ON)
option(PDAAAL_AddressSanitizer "Enables address sanitization during compilation." OFF)
option(PDAAAL_GetDependencies "Fetch external dependencies from web." ON)


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wpedantic -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG -Wall -Wpedantic -fPIC")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(PDAAAL_AddressSanitizer)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -fno-omit-frame-pointer -fsanitize=address")
endif(PDAAAL_AddressSanitizer)

if (PDAAAL_GetDependencies)
    # setup for external imports
    include(ExternalProject)
    set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)

    if (PTRIE_INSTALL_DIR)
        include_directories(${PTRIE_INSTALL_DIR})
    else()
        ExternalProject_add(ptrie-ext
                GIT_REPOSITORY git@github.com:MortenSchou/ptrie.git
                GIT_TAG af7f132f2c3e81f7402be51828f3e1076ff895a1
                #URL https://github.com/petergjoel/ptrie/archive/v1.1.1.zip
                #URL_HASH SHA512=5435d6f8132d273b50eccd721790d559f7dc67ffdada75fec315f4c53705199f31f1ddc80dda0cb86f883db27076fa6d324d28516b442fecce3ffa51213aef4e
                CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION} -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
                )
        file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
    endif()
    # we can now include external libraries
    include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
    link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)
endif (PDAAAL_GetDependencies)


#actual library
add_subdirectory(src)

set(BUILD_TESTING ON)

if(BUILD_TESTING AND PDAAAL_BuildTests)
    #benchmark
    add_subdirectory(benchmark)

    #testing
    add_subdirectory(test)
    enable_testing()
    add_test(NAME TestPDAFactory    COMMAND TestPDAFactory)
    add_test(NAME Weight            COMMAND Weight)
    add_test(NAME WPDA              COMMAND WPDA)
    add_test(NAME PAutomaton        COMMAND PAutomaton)
    add_test(NAME Solver            COMMAND Solver)
    add_test(NAME Reducer           COMMAND Reducer)
    add_test(NAME PDAFactory        COMMAND PDAFactory)
    add_test(NAME fut_set           COMMAND fut_set)
    add_test(NAME NFA               COMMAND NFA)
    add_test(NAME ParsingPDAFactory COMMAND ParsingPDAFactory)
endif()
