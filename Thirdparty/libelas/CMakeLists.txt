# project
cmake_minimum_required (VERSION 2.6)
project (libelas)


SET (LIBELAS_LIB_TYPE SHARED)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})


# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# Set the output directory for the build executables and libraries
IF(WIN32)
    SET(libelas_LIBRARY_OUTPUT_DIRECTORY ${libelas_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
    SET(libelas_LIBRARY_OUTPUT_DIRECTORY ${libelas_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${libelas_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${libelas_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${libelas_RUNTIME_OUTPUT_DIRECTORY})


# sources
FILE(GLOB LIBELAS_SRC_FILES "src/*.cpp" "src/*.h")

INCLUDE_DIRECTORIES(
        ${libelas_SOURCE_DIR}/src
        )

# Include the subdirectories
ADD_LIBRARY(libelas ${LIBELAS_LIB_TYPE}  ${LIBELAS_SRC_FILES})

