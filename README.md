# Computational Geometry

[![CI](https://github.com/alexjmak/Computational-Geometry/actions/workflows/ci.yml/badge.svg)](https://github.com/alexjmak/Computational-Geometry/actions/workflows/ci.yml)

A C++23 computational geometry toolkit with a reusable core library, command-line
tools, a Qt GUI, pybind11 Python bindings, YAML geometry I/O, GoogleTest tests,
and optional Doxygen documentation.

## Features

- Exact rational geometry primitives for points, segments, polygons, rectangles,
  and DCEL structures.
- Convex hull construction.
- Line segment intersection with both line-sweep and brute-force algorithms.
- YAML document format organized into named geometry layers.
- Random point and segment generation for repeatable examples and tests.
- Qt-based visualization executable, `cgeom-gui`.
- Python module, `cgeom`, generated from the C++ library with type stubs.

## Requirements

- CMake 3.14 or newer
- Ninja
- A C++23 compiler
- Boost
- Qt6 Widgets, Charts, and Concurrent
- Python 3
- `pybind11-stubgen`
- Doxygen, optional and only needed for local documentation generation

On Ubuntu, the project dependencies used by CI are:

```bash
sudo apt-get update
sudo apt-get install -y \
  cmake \
  clang-tidy \
  g++ \
  libboost-all-dev \
  libqt6charts6-dev \
  ninja-build \
  python3-dev \
  python3-pip \
  qt6-base-dev

python3 -m pip install pybind11-stubgen
```

## Build

Configure and build a debug tree:

```bash
cmake -S . -B build-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug
```

For a release build, use a separate build directory:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

The build produces:

- `cgeom`, the CLI executable
- `cgeom-gui`, the Qt GUI executable
- `cgeom`, the Python extension module in the build directory
- GoogleTest binaries under the build tree

## Test

Run the CTest suite from the debug build:

```bash
QT_QPA_PLATFORM=offscreen ctest --test-dir build-debug --output-on-failure
```

The `QT_QPA_PLATFORM=offscreen` environment variable lets GUI tests run in a
headless environment.

CI runs Debug and Release builds on Ubuntu and executes the CTest suite.

## CLI Examples

Generate deterministic random segments:

```bash
./build-debug/cgeom random segments random.yaml -count 100 -seed 0
```

Find segment intersections with the line-sweep algorithm:

```bash
./build-debug/cgeom intersection random.yaml output.yaml -algorithm line-sweep
```

Compute a convex hull from the geometry in a document:

```bash
./build-debug/cgeom convex_hull random.yaml hull.yaml
```

Compare generated YAML documents:

```bash
./build-debug/cgeom compare output.yaml output.yaml
```

Other CLI subcommands include `filter` and `script`.

## Python Example

Set `PYTHONPATH` to the build directory so Python can import the compiled module:

```bash
PYTHONPATH=build-debug python3 - <<'PY'
import cgeom as cg

segments = cg.random_segments(count=100, seed=0)

random_document = cg.Document()
random_document.add_layer("random_segments", segments=segments)
cg.save_yaml(random_document, "random.yaml")

intersections = cg.line_segment_intersection(segments)

output_document = cg.load_yaml("random.yaml")
output_document.add_layer("intersections", points=intersections)
cg.save_yaml(output_document, "output.yaml")

print(f"Found {len(intersections)} intersections")
PY
```

## YAML Format

Geometry files are YAML documents with ordered `layers`. Each layer has a
required `name` and can contain `points`, `segments`, and `polygons`.

```yaml
layers:
  - name: sample_points
    points:
      - [0, 0]
      - [10, 0]
      - [10, 10]
      - [0, 10]

  - name: sample_segments
    segments:
      - [[0, 0], [10, 10]]
      - [[0, 10], [10, 0]]

  - name: sample_polygon
    polygons:
      - outer:
          - [0, 0]
          - [10, 0]
          - [10, 10]
          - [0, 10]
```

Rational coordinates can also be represented as `[numerator, denominator]`
inside a coordinate pair.

## GUI

Build the project, then run the Qt viewer against a YAML document:

```bash
./build-debug/cgeom-gui output.yaml
```

## Documentation

If Doxygen is installed, generate local HTML documentation with:

```bash
cmake --build build-debug --target docs
```

Generated documentation is written under `docs/`, which is ignored by Git.

## Project Layout

- `src/geometry`: core primitives and predicates
- `src/algorithms`: convex hull and line segment intersection algorithms
- `src/io`: YAML document model and serialization
- `src/cli`: command-line parser and operations
- `src/gui`: Qt visualization components
- `src/python`: pybind11 bindings for the C++ library
- `tests`: GoogleTest suites and a Python binding example
