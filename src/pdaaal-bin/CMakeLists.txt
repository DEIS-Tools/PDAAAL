cmake_minimum_required(VERSION 3.14)
project(PDAAAL-bin VERSION ${PDAAAL_VERSION} LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.10.0.tar.gz
    URL_HASH SHA512=1531780110260ea4cd8fbe79d860f1ea46fa8377d520669f938ddb5f8a4b11fe550fc7d2d978a281d5ddcc511447d160a2f68c06583672496c57c6672a4036a1
)
FetchContent_GetProperties(json)
if(NOT json_POPULATED)
    FetchContent_Populate(json)
    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# Local definition of source files.
set(PDAAAL_BIN_SOURCE_FILES
    main.cpp
    parsing/Parsing.cpp
)
# The following writes a Git hash at build time to a header file that is then included by main.cpp
# We also include the PDAAAL_VERSION number as defined in the top-level cmake file.
if(DEFINED ENV{GITHUB_SHA}) # Workaround for Github CI workflow, where the current directory (for some odd reason) is not a git repository.
    add_custom_command(OUTPUT version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY(x) #x" > version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY_MACRO(x) STRINGIFY(x)" >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo_append "#define PDAAAL_GIT_HASH " >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo "$ENV{GITHUB_SHA}" >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_GIT_HASH_STR STRINGIFY_MACRO(PDAAAL_GIT_HASH)" >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_VERSION ${PDAAAL_VERSION}" >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_VERSION_STR STRINGIFY_MACRO(PDAAAL_VERSION)" >> version.h
            DEPENDS pdaaal ${PDAAAL_BIN_SOURCE_FILES} ${HEADER_FILES}
            VERBATIM)
else()
    find_package(Git)
    add_custom_command(OUTPUT version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY(x) #x" > version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY_MACRO(x) STRINGIFY(x)" >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo_append "#define PDAAAL_GIT_HASH " >> version.h
            COMMAND ${GIT_EXECUTABLE} rev-parse HEAD >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_GIT_HASH_STR STRINGIFY_MACRO(PDAAAL_GIT_HASH)" >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_VERSION ${PDAAAL_VERSION}" >> version.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_VERSION_STR STRINGIFY_MACRO(PDAAAL_VERSION)" >> version.h
            DEPENDS pdaaal ${PDAAAL_BIN_SOURCE_FILES} ${HEADER_FILES}
            VERBATIM)
endif()
add_custom_target(pdaaal_generate_version DEPENDS "version.h")

# Executable for the stand-alone binary version.
add_executable(pdaaal-bin)
target_sources(pdaaal-bin
        PRIVATE ${PDAAAL_BIN_SOURCE_FILES}
                ${HEADER_FILES}
                ${CMAKE_CURRENT_BINARY_DIR}/version.h
)
target_link_libraries(pdaaal-bin
        PRIVATE pdaaal
                Boost::program_options
                nlohmann_json::nlohmann_json
)
target_include_directories(pdaaal-bin PRIVATE ${CMAKE_CURRENT_BINARY_DIR}) # version.h is located here
target_include_directories(pdaaal-bin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_dependencies(pdaaal-bin pdaaal_generate_version)
set_target_properties(pdaaal-bin PROPERTIES OUTPUT_NAME pdaaal) # Name the executable file pdaaal.
