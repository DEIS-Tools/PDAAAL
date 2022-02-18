add_definitions (-DBOOST_TEST_DYN_LINK)

# Tests for pdaaal (library)
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
    AutomatonPath_test.cpp
)

foreach(test_source_file ${PDAAAL_test_sources})
    get_filename_component(exename ${test_source_file} NAME_WE)
    add_executable(${exename} ${test_source_file})
    target_link_libraries(${exename} PRIVATE Boost::unit_test_framework pdaaal)
    target_compile_options(${exename} PRIVATE -Wall -Wextra)
endforeach()

# Tests for pdaaal-bin (stand-alone binary)
if (PDAAAL_Build_Main) # We need the dependencies for pdaaal-bin to test it.
    set(PDAAAL_bin_test_sources
            Verification_test.cpp
    )
    foreach(test_source_file ${PDAAAL_bin_test_sources})
        get_filename_component(exename ${test_source_file} NAME_WE)
        add_executable(${exename} ${test_source_file})
        target_link_libraries(${exename} PRIVATE Boost::unit_test_framework pdaaal nlohmann_json::nlohmann_json)
        target_include_directories(${exename} PRIVATE ${CMAKE_SOURCE_DIR}/src/pdaaal-bin)
        target_compile_options(${exename} PRIVATE -Wall -Wextra)
    endforeach()

endif()
