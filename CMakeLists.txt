cmake_minimum_required(VERSION 3.14)
project(ComputationalGeometry)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(ENABLE_CLANG_TIDY "Run clang-tidy during builds" ON)

include(FetchContent)
include(CTest)

find_package(Boost CONFIG REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Charts Concurrent)
find_package(Doxygen)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

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

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(pybind11 yaml-cpp googletest)

if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE clang-tidy
        PATHS /usr/local/opt/llvm/bin
    )
    set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE} --use-color)

    if(APPLE)
        list(APPEND CMAKE_CXX_CLANG_TIDY --extra-arg=-isystem${Boost_INCLUDE_DIRS})
    endif()
endif()

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
    src/algorithms/assemble.cpp
    src/algorithms/convex_hull.cpp
    src/algorithms/horizontal_ray_query.cpp
    src/algorithms/line_segment_intersection.cpp
    src/algorithms/overlay.cpp
    src/algorithms/sweep_line.cpp
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

add_library(
    gui_core
    STATIC
    src/gui/controller.cpp
    src/gui/plot.cpp
)

target_include_directories(
    gui_core
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${Qt6Widgets_INCLUDE_DIRS}
    ${Qt6Charts_INCLUDE_DIRS}
    ${Qt6Core_INCLUDE_DIRS}
)

target_link_libraries(
    gui_core
    PUBLIC
    geometry_core
    Qt6::Widgets
    Qt6::Charts
    Qt6::Concurrent
)

set_target_properties(
    gui_core
    PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

add_executable(
    cgeom-gui
    src/gui/main.cpp
)

target_link_libraries(
    cgeom-gui
    PRIVATE
    gui_core
)

pybind11_add_module(
    cgeom_python
    src/python/bindings.cpp
    src/python/geometry_bindings.cpp
    src/python/document_bindings.cpp
    src/python/gui_bindings.cpp
    src/python/algorithm_bindings.cpp
)

set_target_properties(
    cgeom_python
    PROPERTIES
    OUTPUT_NAME cgeom
)

target_link_libraries(
    cgeom_python
    PRIVATE
    gui_core
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/cgeom.pyi
    COMMAND ${CMAKE_COMMAND} -E env
            PYTHONPATH=$<TARGET_FILE_DIR:cgeom_python>
            ${Python3_EXECUTABLE} -m pybind11_stubgen
            cgeom
            -o ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS cgeom_python
    COMMENT "Generating Python type hints for cgeom"
    VERBATIM
)

add_custom_target(
    copy_cgeom_pyi ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/cgeom.pyi
)

include(GoogleTest)

add_executable(
    geometry_tests
    tests/geometry_tests.cpp
)

set_target_properties(
    geometry_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests
)

target_link_libraries(
    geometry_tests
    PRIVATE
    geometry_core
    GTest::gtest_main
)

gtest_discover_tests(geometry_tests)

add_executable(
    algorithm_tests
    tests/algorithm_tests.cpp
)

set_target_properties(
    algorithm_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests
)

target_link_libraries(
    algorithm_tests
    PRIVATE
    geometry_core
    GTest::gtest_main
)

gtest_discover_tests(algorithm_tests)

add_executable(
    gui_tests
    tests/gui_tests.cpp
)

set_target_properties(
    gui_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests
)

target_link_libraries(
    gui_tests
    PRIVATE
    gui_core
    GTest::gtest
)

gtest_discover_tests(
    gui_tests
    PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

add_custom_target(
    tests
    DEPENDS
    geometry_tests
    algorithm_tests
    gui_tests
)

if(DOXYGEN_FOUND)
    add_custom_target(
        docs
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_SOURCE_DIR}/docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Cleaning and generating Doxygen documentation"
        VERBATIM
    )
endif()
