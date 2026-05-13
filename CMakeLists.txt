cmake_minimum_required(VERSION 3.14)
project(ComputationalGeometry)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

find_package(Boost CONFIG REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Charts Concurrent)
find_package(Doxygen)

set(PYBIND11_FINDPYTHON ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)

FetchContent_Declare(
    yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(pybind11 yaml-cpp)

if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

find_program(CLANG_TIDY_EXE clang-tidy
    PATHS /usr/local/opt/llvm/bin
)
set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE} --extra-arg=-isystem${Boost_INCLUDE_DIRS})

add_library(
    geometry_core
    STATIC
    src/geometry/vector.cpp
    src/geometry/segment.cpp
    src/geometry/polygon.cpp
    src/geometry/predicates.cpp
    src/geometry/intersection.cpp
    src/geometry/random.cpp
    src/geometry/dcel.cpp
    src/algorithms/convex_hull.cpp
    src/algorithms/line_segment_intersection.cpp
    src/io/document.cpp
    src/io/geometry_io.cpp
)

target_include_directories(
    geometry_core
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

target_include_directories(
    geometry_core
    SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(
    geometry_core
    PUBLIC
    yaml-cpp::yaml-cpp
)

set_target_properties(
    geometry_core
    PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

add_executable(
    cgeom
    src/cli/main.cpp
    src/cli/arg_parser.cpp
    src/cli/operations.cpp
)

target_link_libraries(
    cgeom
    PRIVATE
    geometry_core
)

add_executable(
    cgeom-gui
    src/gui/main.cpp
    src/gui/controller.cpp
    src/gui/plot.cpp
)

target_link_libraries(
    cgeom-gui
    PRIVATE
    geometry_core
    Qt6::Widgets
    Qt6::Charts
    Qt6::Concurrent
)

pybind11_add_module(
    cgeom_python
    src/python/bindings.cpp
)

set_target_properties(
    cgeom_python
    PROPERTIES
    OUTPUT_NAME cgeom
)

target_link_libraries(
    cgeom_python
    PRIVATE
    geometry_core
)

if(DOXYGEN_FOUND)
    add_custom_target(
        docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating Doxygen documentation"
        VERBATIM
    )
endif()
