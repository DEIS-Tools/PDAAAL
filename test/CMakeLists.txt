find_package (Boost COMPONENTS unit_test_framework REQUIRED)
include_directories (${TEST_SOURCE_DIR}/src
        ${Boost_INCLUDE_DIRS}
        ${PDAAAL_SOURCE_DIR}
        )
add_definitions (-DBOOST_TEST_DYN_LINK)

add_executable (TestPDAFactory TestPDAFactory.h TestPDAFactory.cpp testmain.cpp)
add_executable (Weight Weight_test.cpp)
add_executable (WPDA PDA_test.cpp)
add_executable (PAutomaton PAutomaton_test.cpp)
add_executable (Solver Solver_test.cpp)
add_executable (Reducer Reducer_test.cpp)
add_executable (PDAFactory PDAFactory_test.cpp)
add_executable (fut_set fut_set_test.cpp)
add_executable (NFA NFA_test.cpp)

target_link_libraries(TestPDAFactory ${Boost_LIBRARIES} pdaaal)
target_link_libraries(Weight ${Boost_LIBRARIES} pdaaal)
target_link_libraries(WPDA ${Boost_LIBRARIES} pdaaal)
target_link_libraries(PAutomaton ${Boost_LIBRARIES} pdaaal)
target_link_libraries(Solver ${Boost_LIBRARIES} pdaaal)
target_link_libraries(Reducer ${Boost_LIBRARIES} pdaaal)
target_link_libraries(PDAFactory ${Boost_LIBRARIES} pdaaal)
target_link_libraries(fut_set ${Boost_LIBRARIES} pdaaal)
target_link_libraries(NFA ${Boost_LIBRARIES} pdaaal)
