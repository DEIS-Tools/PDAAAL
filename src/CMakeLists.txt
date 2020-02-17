cmake_minimum_required(VERSION 3.7)
project(PDAAAL C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(pdaaal model/PDA.h engine/PAutomaton.h engine/PAutomaton.cpp model/PDA.cpp ${HEADER_FILES})
target_include_directories (pdaaal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS pdaaal
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install (FILES  engine/PAutomaton.h engine/Solver.h model/NFA.h model/PDA.h model/PDAFactory.h model/SimplePDAFactory.h model/TypedPDA.h DESTINATION include/pdaaal)

