cmake_minimum_required(VERSION 3.7)
project(PDAAAL C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Boost 1.66 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_library(pdaaal ${HEADER_FILES} pdaaal/PDA.cpp pdaaal/Reducer.cpp)

if (NOT PTRIE_INSTALL_DIR)
    add_dependencies(pdaaal ptrie-ext)
endif()

target_include_directories (pdaaal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS pdaaal
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install (FILES pdaaal/vector_set.h pdaaal/fut_set.h pdaaal/NFA.h pdaaal/Weight.h pdaaal/PDA.h pdaaal/PDAAdapter.h pdaaal/PDAFactory.h pdaaal/SimplePDAFactory.h pdaaal/TypedPDA.h pdaaal/PAutomaton.h pdaaal/Solver.h pdaaal/SolverAdapter.h pdaaal/Reducer.h DESTINATION include/pdaaal)
