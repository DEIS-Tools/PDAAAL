cmake_minimum_required(VERSION 3.14)

project(PDAAAL-lib VERSION ${PDAAAL_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)
add_library(pdaaal)
add_library(pdaaal::pdaaal ALIAS pdaaal)

include(GNUInstallDirs) # With GNUInstallDirs we use platform-independent macros to get the correct install directory names.  (CMAKE_INSTALL_BINDIR, CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_INCLUDEDIR)

# Here we define the source files for building pdaaal, and that the header files can also be used when building dependents.
target_sources(pdaaal
    PRIVATE pdaaal/internal/PDA.cpp
            pdaaal/internal/Reducer.cpp
            pdaaal/parsing/NfaParser.cpp
    PUBLIC  ${HEADER_FILES}
)
target_compile_features(pdaaal PUBLIC cxx_std_17) # Require C++17 features.
if (MSVC)
    target_compile_options(pdaaal PRIVATE /W4)
else()
    target_compile_options(pdaaal PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_include_directories(pdaaal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
) # The correct include directory depends on whether we are building or installing pdaaal.
#target_link_libraries(pdaaal PUBLIC m) # Uncomment this if we need to include <math.h>

if (PDAAAL_GetDependencies)
    include(FetchContent)
    FetchContent_Declare(ptrie
        GIT_REPOSITORY https://github.com/petergjoel/ptrie.git
        GIT_TAG f45e1a39524a95a28684aa75b169afa476ac025c
    )
    FetchContent_MakeAvailable(ptrie)

    # PEGTL (Parsing Expression Grammar Template Library) is a modern header-only parser combinator library.
    FetchContent_Declare(pegtl
            URL https://github.com/taocpp/PEGTL/archive/refs/tags/3.2.5.tar.gz
            URL_HASH SHA512=e531eaeef614d822e4bddbc6662fbe116cc1536fa308109f28ce5433607e6102f4e754a31094f9c349e4319914da6c83450dd2e8fa10dcfc3eee5a5dca547c14
            )
    FetchContent_MakeAvailable(pegtl)

    # Abseil is a collection of fundamental libraries originally developed by Google.
    # We use their hash framework.
    FetchContent_Declare(abseil
            URL https://github.com/abseil/abseil-cpp/archive/refs/tags/20211102.0.tar.gz
            URL_HASH SHA512=fed68aa434c02ec6faa9d1c81f1ad35b60ec024b44957e2e0ac31e6075e385c06a3e1b616afeb4bb7c2413191fd7827d82e1f9f0796b52ed21fb2c41dd9031cf
            ) # Abseil suggests to 'live at head', but for now we use their lts snapshots to make sure we can always recreate any specific version of pdaaal.
    # We could use FetchContent_MakeAvailable(abseil), but that doesn't support EXCLUDE_FROM_ALL, which excludes the abseil targets from the default ALL make target.
    FetchContent_GetProperties(abseil)
    if(NOT abseil_POPULATED)
        FetchContent_Populate(abseil)
        set(ABSL_ENABLE_INSTALL ON)
        set(ABSL_PROPAGATE_CXX_STD ON)
        add_subdirectory(${abseil_SOURCE_DIR} ${abseil_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()

    FetchContent_Declare(nlohmann_json
            URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz
            URL_HASH SHA512=feb7dbdd1f1b6318131821f55d1411f0435262601c65eca368ccf3418750fd6348a37a3cd9d0694b31ccacfc91d02fbbecf94af52429005f8898490a5233c37d
            )
    FetchContent_GetProperties(nlohmann_json)
    if(NOT nlohmann_json_POPULATED)
        FetchContent_Populate(nlohmann_json)
        set(JSON_Install ON)
        add_subdirectory(${nlohmann_json_SOURCE_DIR} ${nlohmann_json_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()

endif (PDAAAL_GetDependencies)

# Define library dependencies.
target_link_libraries(pdaaal PUBLIC Boost::headers ptrie pegtl absl::hash nlohmann_json::nlohmann_json)

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

# Creates PDAAAL_EXPORT macro in pdaaal_export.h for platform-independent control of exported classes.
# Currently this macro is empty, since pdaaal is a static library, but we keep it as part of standard cmake setup.
include(GenerateExportHeader)
generate_export_header(pdaaal)
install(FILES "${PROJECT_BINARY_DIR}/pdaaal_export.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CMakePackageConfigHelpers)
# pdaaal-config.cmake.in defines dependencies that should be (transitively) available in pdaaal's parent project.
configure_package_config_file(pdaaal-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/pdaaal-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdaaal
)
# Setup creation of versioned package. PDAAAL_VERSION is defined in the top-level cmake file.
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/pdaaal-config-version.cmake
    VERSION ${PDAAAL_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/pdaaal-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/pdaaal-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pdaaal)
