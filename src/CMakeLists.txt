cmake_minimum_required(VERSION 3.14)

add_subdirectory(include)

if (PDAAAL_Build_Main)
    add_subdirectory(pdaaal-bin)
endif()
