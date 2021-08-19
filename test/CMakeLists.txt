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

enable_testing()
foreach(test_source_file ${PDAAAL_test_sources})
    get_filename_component(exename ${test_source_file} NAME_WE)
    add_executable(${exename} ${test_source_file})
    target_link_libraries(${exename} PRIVATE Boost::unit_test_framework pdaaal)
    target_compile_options(${exename} PRIVATE -Wall -Wextra)
    add_test(NAME ${exename} COMMAND ${exename})
endforeach()

set(PDAAAL_bin_test_sources
        Verification_test.cpp
)
foreach(test_source_file ${PDAAAL_bin_test_sources})
    get_filename_component(exename ${test_source_file} NAME_WE)
    add_executable(${exename} ${test_source_file})
    target_link_libraries(${exename} PRIVATE Boost::unit_test_framework pdaaal nlohmann_json::nlohmann_json)
    target_include_directories(${exename} PRIVATE ${CMAKE_SOURCE_DIR}/src/pdaaal-bin)
    target_compile_options(${exename} PRIVATE -Wall -Wextra)
    add_test(NAME ${exename} COMMAND ${exename})
endforeach()
