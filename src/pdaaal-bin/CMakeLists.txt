cmake_minimum_required(VERSION 3.14)
project(PDAAAL-bin CXX)

include(FetchContent)
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent # Use this rather than https://github.com/nlohmann/json to reduce download size.
#    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.9.1
)
FetchContent_GetProperties(json)
if(NOT json_POPULATED)
    FetchContent_Populate(json)
    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

set(PDAAAL_BIN_SOURCE_FILES
    main.cpp
    parsing/Parsing.cpp
)
# The following writes a Git hash at build time to a header file that is then included by main.cpp
if(DEFINED ENV{GITHUB_SHA}) # Workaround for Github CI workflow, where the current directory (for some odd reason) is not a git repository.
    add_custom_command(OUTPUT git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo_append "#define PDAAAL_GIT_HASH " > git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "$ENV{GITHUB_SHA}" >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY(x) #x" >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY_MACRO(x) STRINGIFY(x) " >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_GIT_HASH_STR STRINGIFY_MACRO(PDAAAL_GIT_HASH) " >> git_hash.h
            DEPENDS pdaaal ${PDAAAL_BIN_SOURCE_FILES}
            VERBATIM)
else()
    find_package(Git)
    add_custom_command(OUTPUT git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo_append "#define PDAAAL_GIT_HASH " > git_hash.h
            COMMAND ${GIT_EXECUTABLE} rev-parse HEAD >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY(x) #x" >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define STRINGIFY_MACRO(x) STRINGIFY(x) " >> git_hash.h
            COMMAND ${CMAKE_COMMAND} -E echo "#define PDAAAL_GIT_HASH_STR STRINGIFY_MACRO(PDAAAL_GIT_HASH) " >> git_hash.h
            DEPENDS pdaaal ${PDAAAL_BIN_SOURCE_FILES}
            VERBATIM)
endif()
add_custom_target(generate_hash DEPENDS "git_hash.h")

# Executable for the stand-alone binary version.
add_executable(pdaaal-bin)
target_sources(pdaaal-bin PRIVATE ${PDAAAL_BIN_SOURCE_FILES} ${HEADER_FILES} ${CMAKE_CURRENT_BINARY_DIR}/git_hash.h)
target_link_libraries(pdaaal-bin PRIVATE pdaaal Boost::program_options nlohmann_json::nlohmann_json)
target_include_directories(pdaaal-bin PRIVATE ${CMAKE_CURRENT_BINARY_DIR}) # git_hash.h is located here
target_include_directories(pdaaal-bin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_dependencies(pdaaal-bin generate_hash)
set_target_properties(pdaaal-bin PROPERTIES OUTPUT_NAME pdaaal)
