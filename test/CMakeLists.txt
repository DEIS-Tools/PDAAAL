find_package (Boost COMPONENTS unit_test_framework REQUIRED)
include_directories (${TEST_SOURCE_DIR}/src
        ${Boost_INCLUDE_DIRS}
        ${PDAAAL_SOURCE_DIR}
        )
add_definitions (-DBOOST_TEST_DYN_LINK)

# add_executable (Test1 TestPDAFactory.h TestPDAFactory.cpp testmain.cpp)
add_executable (WPDA WPDA_test.cpp)
add_executable (PAutomaton PAutomaton_test.cpp)

# target_link_libraries(Test1 ${Boost_LIBRARIES} pdaaal)
target_link_libraries(WPDA ${Boost_LIBRARIES} pdaaal)
target_link_libraries(PAutomaton ${Boost_LIBRARIES} pdaaal)