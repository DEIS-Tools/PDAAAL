cmake_minimum_required(VERSION 3.14)

# The source code is divided into the library (src/include) and the code for the stand-alone binary (src/pdaaal-bin).

add_subdirectory(include)

if (PDAAAL_Build_Main)
    add_subdirectory(pdaaal-bin)
endif()
