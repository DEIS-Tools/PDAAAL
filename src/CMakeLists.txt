cmake_minimum_required(VERSION 3.7)
project(PDAAAL C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(pdaaal ${HEADER_FILES} pdaaal/PDA.cpp pdaaal/Reducer.cpp)

if (NOT PTRIE_INSTALL_DIR)
    add_dependencies(pdaaal ptrie-ext)
endif()

target_include_directories (pdaaal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(DEFINED ENV{GITHUB_SHA}) # Workaround for Github CI workflow, where the current directory (for some odd reason) is not a git repository.
    add_custom_command(OUTPUT git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo_append "#define PDAAAL_GIT_HASH " > git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "$ENV{GITHUB_SHA}" >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY(x) #x" >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY_MACRO(x) STRINGIFY(x) " >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_GIT_HASH_STR STRINGIFY_MACRO(PDAAAL_GIT_HASH) " >> git_hash.h
            DEPENDS pdaaal main.cpp
            VERBATIM)
else()
    find_package(Git)
    add_custom_command(OUTPUT git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo_append "#define PDAAAL_GIT_HASH " > git_hash.h
            COMMAND ${GIT_EXECUTABLE} rev-parse HEAD >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY(x) #x" >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY_MACRO(x) STRINGIFY(x) " >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_GIT_HASH_STR STRINGIFY_MACRO(PDAAAL_GIT_HASH) " >> git_hash.h
            DEPENDS pdaaal main.cpp pdaaal/parsing/Parsing.cpp pdaaal/parsing/JsonParser.cpp
            VERBATIM)
endif()
add_custom_target(generate_hash DEPENDS "git_hash.h")

find_package(Boost 1.66 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(pdaaal-bin main.cpp pdaaal/parsing/Parsing.cpp pdaaal/parsing/JsonParser.cpp)
target_link_libraries(pdaaal-bin PRIVATE pdaaal ${Boost_LIBRARIES})
add_dependencies(pdaaal-bin generate_hash)
set_target_properties(pdaaal-bin PROPERTIES OUTPUT_NAME pdaaal)

install(TARGETS pdaaal pdaaal-bin
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
# install(FILES ${HEADER_FILES} DESTINATION include/pdaaal)  # Maybe use this simpler line instead of the following.
install (FILES pdaaal/utils/vector_set.h pdaaal/utils/fut_set.h pdaaal/NFA.h pdaaal/Weight.h pdaaal/PDA.h
        pdaaal/PDAFactory.h pdaaal/SolverInstance.h
        pdaaal/ParsingPDAFactory.h
        pdaaal/Refinement.h
        pdaaal/utils/std20.h
        pdaaal/AbstractionMapping.h pdaaal/AbstractionPDA.h pdaaal/AbstractionPAutomaton.h pdaaal/CegarPdaFactory.h pdaaal/utils/ptrie_interface.h
        pdaaal/SimplePDAFactory.h pdaaal/TypedPDA.h pdaaal/PAutomaton.h pdaaal/Solver.h pdaaal/Reducer.h DESTINATION include/pdaaal)
