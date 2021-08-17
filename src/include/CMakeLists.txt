cmake_minimum_required(VERSION 3.14)

project(PDAAAL-lib VERSION ${PDAAAL_VERSION} LANGUAGES CXX)

#set(CMAKE_CXX_VISIBILITY_PRESET hidden)  ## Not sure about these
#set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)  |
add_library(pdaaal)
add_library(pdaaal::pdaaal ALIAS pdaaal)
#include(GenerateExportHeader)             |
#generate_export_header(pdaaal)            |.

include(GNUInstallDirs) # With GNUInstallDirs we use platform-independent macros to get the correct install directory names.  (CMAKE_INSTALL_BINDIR, CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_INCLUDEDIR)

# Here we define the source files for building pdaaal, and that the header files can also be used when building dependents.
target_sources(pdaaal
    PRIVATE pdaaal/PDA.cpp
            pdaaal/Reducer.cpp
            pdaaal/parsing/NfaParser.cpp
    PUBLIC  ${HEADER_FILES}
)
target_compile_features(pdaaal PUBLIC cxx_std_17) # Require C++17 features.
target_compile_definitions(pdaaal PUBLIC $<$<CONFIG:Release>:NDEBUG>) # We can use "#ifndef NDEBUG\n <debug only code> \n#endif" to define code that is only used in debug mode.
target_include_directories(pdaaal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
) # The correct include directory depends on whether we are building or installing pdaaal.
#target_link_libraries(pdaaal PUBLIC m) # Uncomment this if we need to include <math.h>

if (PDAAAL_GetDependencies)
    # TODO: Prefer to use FetchContent over ExternalProject, and make ptrie available for parent projects too.
    # setup for external imports
    include(ExternalProject)
    set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)

    ExternalProject_add(ptrie-ext
            URL https://github.com/MortenSchou/ptrie/archive/refs/tags/v1.1.2beta.zip
            URL_HASH SHA512=7dbfd54839509187d27e2ddea34c253ac1f26712012c29eca0d96b9f7bcf9844bdc3b40955aa98e391344745b734aa2f28cf115572b4b0ab2873582b32dfe228
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION} -DBUILD_TESTING=OFF
            )
    file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
    # we can now include external libraries
    add_dependencies(pdaaal ptrie-ext)
    target_include_directories(pdaaal PUBLIC
            $<BUILD_INTERFACE:${EXTERNAL_INSTALL_LOCATION}/include>
            $<INSTALL_INTERFACE:external/include>)


    include(FetchContent)
    # PEGTL (Parsing Expression Grammar Template Library) is a modern header-only parser combinator library.
    FetchContent_Declare(pegtl
            URL https://github.com/taocpp/PEGTL/archive/refs/tags/3.2.0.zip
            URL_HASH SHA512=b24d5b77de864fc79fd60c9d17e040dfc5d6a400c7282eac62cb87c1a53c6f6f932c62b88a296f487c6155e98d04f1b5e69059589db99e95c313766ffbbc3913
            )
    FetchContent_MakeAvailable(pegtl)

    # Abseil is a collection of fundamental libraries originally developed by Google.
    # We use their hash framework.
    FetchContent_Declare(abseil
            GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
            GIT_TAG origin/lts_2021_03_24
            ) # Abseil suggests to 'live at head', but for now we use their lts snapshots to make sure we can always recreate any specific version of pdaaal.
    set(ABSL_ENABLE_INSTALL ON)
    # We could use FetchContent_MakeAvailable(abseil), but that doesn't support EXCLUDE_FROM_ALL, which excludes the abseil targets from the default ALL make target.
    FetchContent_GetProperties(abseil)
    if(NOT abseil_POPULATED)
        FetchContent_Populate(abseil)
        add_subdirectory(${abseil_SOURCE_DIR} ${abseil_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
endif (PDAAAL_GetDependencies)

# Define library dependencies.
target_link_libraries(pdaaal PUBLIC Boost::headers pegtl absl::hash)

# Define which directories to install with the pdaaal library.
install(DIRECTORY pdaaal/ pdaaal/parsing/ pdaaal/utils/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/pdaaal)

# More or less boilerplate stuff that ensures everything is installed correctly
install(TARGETS  pdaaal
        EXPORT   pdaaal-targets
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT      pdaaal-targets
        FILE        pdaaal-targets.cmake
        NAMESPACE   pdaaal::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdaaal)
# pdaaal-config.cmake.in defines dependencies that should be (transitively) available in pdaaal's parent project.
configure_file(pdaaal-config.cmake.in pdaaal-config.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pdaaal-config.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdaaal)
# Setup creation of versioned package. PDAAAL_VERSION is defined in the top-level cmake file.
include(CMakePackageConfigHelpers)
write_basic_package_version_file("pdaaal-config-version.cmake"
    VERSION ${PDAAAL_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES "pdaaal-config-version.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdaaal)
