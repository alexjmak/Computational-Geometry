from pathlib import Path

import cgeom as cg


OUTPUT_DIR = Path("examples/output")


def donut_island_segments() -> list[cg.Segment]:
    segments = []
    rectangles = [
        cg.Rectangle(cg.Point(0, 0), cg.Point(10, 10)),
        cg.Rectangle(cg.Point(3, 3), cg.Point(7, 7)),
        cg.Rectangle(cg.Point(4, 4), cg.Point(6, 6)),
    ]
    for rectangle in rectangles:
        segments.extend(rectangle.ring().segments())
    return segments


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "assembled_polygons.yaml"

    segments = donut_island_segments()
    polygons = cg.assemble_polygons(segments)

    document = cg.Document()
    document.add_layer("input_segments", segments=segments)
    document.add_layer("assembled_polygons", polygons=polygons)
    cg.save_yaml(document, str(output_path))

    print(f"Assembled polygons: {len(polygons)}")
    for i, polygon in enumerate(polygons):
        print(
            f"Polygon {i}: outer_area={polygon.outer_ring.area()}, "
            f"holes={len(polygon.inner_rings)}, area={polygon.area()}"
        )
    print(f"Wrote {output_path}")

    plot = cg.Plot("assembled polygons", "x", "y")
    plot.add_segments(segments, color="gray", line_width=1, show_arrows=False)
    for polygon in polygons:
        plot.add_polygon(polygon)
        for ring in polygon.rings():
            plot.add_segments(ring.segments(), show_arrows=True)
    plot.show()


if __name__ == "__main__":
    main()
