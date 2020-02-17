cmake_minimum_required(VERSION 3.7)
project(PDAAAL C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(pdaaal pdaaal/PDA.h pdaaal/PAutomaton.h pdaaal/PAutomaton.cpp pdaaal/PDA.cpp ${HEADER_FILES})
add_dependencies(pdaaal ptrie-ext)
target_include_directories (pdaaal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS pdaaal
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install (FILES pdaaal/PAutomaton.h pdaaal/Solver.h pdaaal/NFA.h pdaaal/PDA.h pdaaal/PDAFactory.h pdaaal/SimplePDAFactory.h pdaaal/TypedPDA.h DESTINATION include/pdaaal)

