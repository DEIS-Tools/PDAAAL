add_definitions (-DBOOST_TEST_DYN_LINK)
set(PDAAAL_test_sources
    Weight_test.cpp
    PDA_test.cpp
    PAutomaton_test.cpp
    Solver_test.cpp
    Reducer_test.cpp
    PDAFactory_test.cpp
    fut_set_test.cpp
    NFA_test.cpp
    ParsingPDAFactory_test.cpp
    NfaParser_test.cpp
)

# file(GLOB ...) is used to validate the above list of test_sources
file(GLOB PDAAAL_glob_test_sources RELATIVE ${CMAKE_CURRENT_LIST_DIR} *.cpp)

enable_testing()
foreach(test_source_file ${PDAAAL_test_sources})
    if(${test_source_file} IN_LIST PDAAAL_glob_test_sources)
        list(REMOVE_ITEM PDAAAL_glob_test_sources ${test_source_file})
    else()
        message(SEND_ERROR "File ${test_source_file} is missing from test")
    endif()

    get_filename_component(exename ${test_source_file} NAME_WE)
    add_executable(${exename} ${test_source_file})
    target_link_libraries(${exename} PRIVATE Boost::unit_test_framework pdaaal)
    if(MSVC)
        target_compile_options(${exename} PRIVATE /W4 /WX /utf-8)
    else()
        target_compile_options(${exename} PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME ${exename} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} COMMAND ${exename})
endforeach()

if(PDAAAL_glob_test_sources)
    foreach(ignored_source_file ${PDAAAL_glob_test_sources})
        message(SEND_ERROR "File ${ignored_source_file} in test is ignored")
    endforeach()
endif()

add_subdirectory(pdaaal-bin-test)
