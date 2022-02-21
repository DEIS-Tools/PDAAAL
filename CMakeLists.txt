cmake_minimum_required(VERSION 3.14)

cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0069 NEW)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PDAAAL_VERSION 1.0.0)
project(pdaaal VERSION ${PDAAAL_VERSION} LANGUAGES CXX)

# MAIN_PROJECT CHECK
# Determine if pdaaal is built as a subproject (using add_subdirectory) or if it is the main project
set(PDAAAL_MAIN_PROJECT OFF)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(PDAAAL_MAIN_PROJECT ON)
endif()

option(PDAAAL_BuildTests "Build the unit tests when BUILD_TESTING is enabled." ${PDAAAL_MAIN_PROJECT})
option(PDAAAL_Build_Main "Build the stand-alone PDAAAL binary." ${PDAAAL_MAIN_PROJECT})
option(PDAAAL_AddressSanitizer "Enables address sanitization during compilation." OFF)
option(PDAAAL_GetDependencies "Fetch external dependencies from web." ON)


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall -Wpedantic")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lstdc++")

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
    enable_testing()
    add_test(NAME Weight_test               COMMAND Weight_test)
    add_test(NAME PDA_test                  COMMAND PDA_test)
    add_test(NAME PAutomaton_test           COMMAND PAutomaton_test)
    add_test(NAME Solver_test               COMMAND Solver_test)
    add_test(NAME Reducer_test              COMMAND Reducer_test)
    add_test(NAME PDAFactory_test           COMMAND PDAFactory_test)
    add_test(NAME fut_set_test              COMMAND fut_set_test)
    add_test(NAME NFA_test                  COMMAND NFA_test)
    add_test(NAME ParsingPDAFactory_test    COMMAND ParsingPDAFactory_test)
    add_test(NAME NfaParser_test            COMMAND NfaParser_test)
    add_test(NAME AutomatonPath_test        COMMAND AutomatonPath_test)

    if (PDAAAL_Build_Main) # We need the dependencies for pdaaal-bin to test it.
        add_test(NAME Verification_test         COMMAND Verification_test)
    endif()

endif()
