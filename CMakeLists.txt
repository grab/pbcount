# need to use CLANG with apple silicon build in, otherwise linker error missing arm instructions etc..

PROJECT(cmake_pbcount)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

# SET(CMAKE_CXX_FLAGS "-g -Ofast -static")
IF(APPLE)
  # apple machines cannot static compile because lack of system files
  SET(CMAKE_CXX_FLAGS "-g -Ofast")
  LINK_DIRECTORIES(/opt/homebrew/lib)
  INCLUDE_DIRECTORIES(/opt/homebrew/include)
ELSE()
  SET(CMAKE_CXX_FLAGS "-g -Ofast -static")
ENDIF()

SET(lib_dir ${CMAKE_CURRENT_SOURCE_DIR}/lib)
SET(lib_tar ${lib_dir}.tar)
SET(cudd_dir ${lib_dir}/cudd-3.0.0)

ADD_CUSTOM_COMMAND(
  OUTPUT ${lib_dir}
  DEPENDS ${lib_tar}
  COMMAND tar -xf ${lib_tar} && touch ${lib_dir} && cd ${cudd_dir} && ./configure --enable-silent-rules --enable-obj && make
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

LINK_DIRECTORIES(${cudd_dir}/cudd/.libs)

INCLUDE_DIRECTORIES(${cudd_dir})
INCLUDE_DIRECTORIES(${cudd_dir}/cudd)
INCLUDE_DIRECTORIES(${cudd_dir}/epd)
INCLUDE_DIRECTORIES(${cudd_dir}/mtr)
INCLUDE_DIRECTORIES(${cudd_dir}/st)

INCLUDE_DIRECTORIES(${lib_dir})

INCLUDE_DIRECTORIES(src/interface)

FILE(GLOB cpp_files "src/implementation/*.cpp")

# temporarily excluding incomplete cpp files from compilation (error otherwise)
# LIST(FILTER cpp_files EXCLUDE REGEX ".*pbformula\\.cpp$")

ADD_EXECUTABLE(pbcount ${cpp_files} ${lib_dir})

TARGET_LINK_LIBRARIES(pbcount cudd)
