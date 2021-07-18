set(PDAAAL_bin_test_sources
    Verification_test.cpp
)

# file(GLOB ...) is used to validate the above list of test_sources
file(GLOB PDAAAL_bin_glob_test_sources RELATIVE ${CMAKE_CURRENT_LIST_DIR} *.cpp)

enable_testing()
foreach(test_source_file ${PDAAAL_bin_test_sources})
    if(${test_source_file} IN_LIST PDAAAL_bin_glob_test_sources)
        list(REMOVE_ITEM PDAAAL_bin_glob_test_sources ${test_source_file})
    else()
        message(SEND_ERROR "File ${test_source_file} is missing from test")
    endif()

    get_filename_component(exename ${test_source_file} NAME_WE)
    add_executable(${exename} ${test_source_file})
    target_link_libraries(${exename} PRIVATE Boost::unit_test_framework pdaaal nlohmann_json::nlohmann_json)
    target_include_directories(${exename} PRIVATE ${CMAKE_SOURCE_DIR}/src/pdaaal-bin)
    if(MSVC)
        target_compile_options(${exename} PRIVATE /W4 /WX /utf-8)
    else()
        target_compile_options(${exename} PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME ${exename} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} COMMAND ${exename})
endforeach()

if(PDAAAL_bin_glob_test_sources)
    foreach(ignored_source_file ${PDAAAL_bin_glob_test_sources})
        message(SEND_ERROR "File ${ignored_source_file} in test is ignored")
    endforeach()
endif()