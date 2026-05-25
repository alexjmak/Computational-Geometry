from pathlib import Path

import cgeom as cg


OUTPUT_DIR = Path("examples/output")


def regular_ring_segments() -> list[cg.Segment]:
    outer_square = [
        cg.Point(0, 0),
        cg.Point(6, 0),
        cg.Point(6, 6),
        cg.Point(0, 6),
    ]
    triangle = [
        cg.Point(8, 1),
        cg.Point(12, 1),
        cg.Point(10, 5),
    ]

    return [
        cg.Segment(outer_square[2], outer_square[1]),
        cg.Segment(triangle[0], triangle[1]),
        cg.Segment(outer_square[3], outer_square[0]),
        cg.Segment(triangle[2], triangle[0]),
        cg.Segment(outer_square[1], outer_square[0]),
        cg.Segment(triangle[1], triangle[2]),
        cg.Segment(outer_square[2], outer_square[3]),
    ]


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "assembled_rings.yaml"

    segments = regular_ring_segments()
    rings = cg.assemble_rings(segments)
    ring_polygons = [cg.Polygon(ring) for ring in rings]

    document = cg.Document()
    document.add_layer("input_segments", segments=segments)
    document.add_layer("assembled_rings", polygons=ring_polygons)
    cg.save_yaml(document, str(output_path))

    print(f"Assembled rings: {len(rings)}")
    for i, ring in enumerate(rings):
        print(f"Ring {i}: area={ring.area()}, is_outer={ring.is_outer()}")
    print(f"Wrote {output_path}")

    plot = cg.Plot("assembled rings", "x", "y")
    plot.add_segments(segments, color="gray", line_width=1, show_arrows=False)
    for ring in rings:
        plot.add_ring(ring)
        plot.add_segments(ring.segments(), show_arrows=True)
    plot.show()


if __name__ == "__main__":
    main()
