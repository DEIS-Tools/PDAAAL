find_package (Boost COMPONENTS unit_test_framework REQUIRED)
include_directories (${TEST_SOURCE_DIR}/src
        ${Boost_INCLUDE_DIRS}
        ${PDAAAL_SOURCE_DIR}
        )
add_definitions (-DBOOST_TEST_DYN_LINK)

add_executable (Test1 TestPDAFactory.h TestPDAFactory.cpp testmain.cpp)

target_link_libraries(Test1 ${Boost_LIBRARIES} pdaaal)
