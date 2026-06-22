# Computational Geometry

[![CI](https://github.com/alexjmak/Computational-Geometry/actions/workflows/ci.yml/badge.svg)](https://github.com/alexjmak/Computational-Geometry/actions/workflows/ci.yml)

A C++23 computational geometry toolkit with a reusable core library, DCEL-based
arrangement utilities, polygon boolean operations, a Qt viewer, pybind11 Python
bindings, YAML geometry I/O, GoogleTest tests, and optional Doxygen
documentation.

## Features

- Exact rational geometry primitives for points, segments, polygons, rectangles,
  and DCEL structures.
- Convex hull construction.
- Line segment intersection with both line-sweep and brute-force algorithms.
- Segment planarization and DCEL construction from planar segment graphs.
- Segment overlay with face labels for the left and right input subdivisions.
- Polygon boolean operations: intersection, union, difference, and symmetric
  difference.
- Ring and polygon assembly from segment boundaries.
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
  doxygen \
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

Polygon boolean operations return boundary segments. Feed those segments into
`assemble_polygons` to recover polygon objects:

```bash
PYTHONPATH=build-debug python3 - <<'PY'
import cgeom as cg

left = cg.Rectangle(cg.Point(0, 0), cg.Point(5, 5)).ring().segments()
right = cg.Rectangle(cg.Point(3, 2), cg.Point(8, 6)).ring().segments()

intersection_segments = cg.polygon_and(left, right)
intersection_polygons = cg.assemble_polygons(intersection_segments)

print(f"Intersection polygons: {len(intersection_polygons)}")
print(f"Intersection area: {sum(p.area() for p in intersection_polygons)}")
PY
```

Other exposed algorithms include `convex_hull`, `assemble_rings`,
`assemble_polygons`, `planarize_segments`, `segment_overlay`, `polygon_or`,
`polygon_difference`, and `polygon_xor`.

## Showcase Scripts

The `scripts/` directory contains small Python examples that generate YAML
documents and plots:

- `showcase_geometry_primitives.py`
- `showcase_segment_intersections.py`
- `showcase_convex_hull.py`
- `showcase_assemble_rings.py`
- `showcase_assemble_polygons.py`
- `showcase_overlay.py`
- `showcase_polygon_and.py`
- `showcase_polygon_or.py`
- `showcase_polygon_difference.py`
- `showcase_polygon_xor.py`

Run one with the build directory on `PYTHONPATH`:

```bash
PYTHONPATH=build-debug python3 scripts/showcase_polygon_and.py
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
- `src/algorithms`: convex hull, segment intersection, assembly, overlay, and
  polygon boolean algorithms
- `src/io`: YAML document model and serialization
- `src/gui`: Qt visualization components
- `src/python`: pybind11 bindings for the C++ library
- `tests`: GoogleTest suites and a Python binding example
