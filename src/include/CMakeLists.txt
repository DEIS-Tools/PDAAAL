cmake_minimum_required(VERSION 3.14)
project(PDAAAL-lib CXX)

#set(CMAKE_CXX_VISIBILITY_PRESET hidden)
#set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)
add_library(pdaaal)
add_library(pdaaal::pdaaal ALIAS pdaaal)
#include(GenerateExportHeader)
#generate_export_header(pdaaal)

target_sources(pdaaal
    PRIVATE pdaaal/PDA.cpp
            pdaaal/Reducer.cpp
            pdaaal/parsing/NfaParser.cpp
    PUBLIC  ${HEADER_FILES}
)
target_compile_features(pdaaal PUBLIC cxx_std_17)
target_include_directories(pdaaal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if (PDAAAL_GetDependencies)
    # setup for external imports
    include(ExternalProject)
    set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)

    if (PTRIE_INSTALL_DIR)
        include_directories(${PTRIE_INSTALL_DIR})
    else()
        ExternalProject_add(ptrie-ext
                URL https://github.com/MortenSchou/ptrie/archive/refs/tags/v1.1.2beta.zip
                URL_HASH SHA512=7dbfd54839509187d27e2ddea34c253ac1f26712012c29eca0d96b9f7bcf9844bdc3b40955aa98e391344745b734aa2f28cf115572b4b0ab2873582b32dfe228
                CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION} -DBUILD_TESTING=OFF
                )
        file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
    endif()

#    file(DOWNLOAD
#        https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp
#        ${EXTERNAL_INSTALL_LOCATION}/include/json.hpp
#        EXPECTED_HASH SHA256=7804b38146921d03374549c9e2a5e3acda097814c43caf2b96a0278e58df26e0
#    )
    # we can now include external libraries
    target_include_directories(pdaaal PUBLIC
            $<BUILD_INTERFACE:${EXTERNAL_INSTALL_LOCATION}/include>
            $<INSTALL_INTERFACE:external/include>)

    include(FetchContent)
    FetchContent_Declare(petgl
            URL https://github.com/taocpp/PEGTL/archive/refs/tags/3.2.0.zip
            URL_HASH SHA512=b24d5b77de864fc79fd60c9d17e040dfc5d6a400c7282eac62cb87c1a53c6f6f932c62b88a296f487c6155e98d04f1b5e69059589db99e95c313766ffbbc3913
            )
    FetchContent_MakeAvailable(petgl)

    FetchContent_Declare(abseil
            GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
            GIT_TAG origin/lts_2021_03_24
            )
    set(ABSL_ENABLE_INSTALL ON)
    FetchContent_GetProperties(abseil)
    if(NOT abseil_POPULATED)
        FetchContent_Populate(abseil)
        add_subdirectory(${abseil_SOURCE_DIR} ${abseil_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
endif (PDAAAL_GetDependencies)

if (NOT PTRIE_INSTALL_DIR)
    add_dependencies(pdaaal ptrie-ext)
endif()

target_link_libraries(pdaaal PUBLIC Boost::headers pegtl absl::hash)

include(GNUInstallDirs)
install(DIRECTORY pdaaal/ pdaaal/parsing/ pdaaal/utils/
        DESTINATION pdaaal)
install(TARGETS pdaaal
        EXPORT pdaaal-config
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(EXPORT pdaaal-config
        NAMESPACE pdaaal::
        FILE pdaaal-targets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdaaal)
configure_file(pdaaal-config.cmake.in pdaaal-config.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pdaaal-config.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdaaal)
