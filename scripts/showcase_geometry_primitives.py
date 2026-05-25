from pathlib import Path

import cgeom as cg


OUTPUT_DIR = Path("examples/output")


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "showcase_geometry_primitives.yaml"

    points = [
        cg.Point(0, 0),
        cg.Point(4, 0),
        cg.Point(4, 4),
        cg.Point(0, 4),
        cg.Point(2, 2),
    ]
    diagonals = [
        cg.Segment(cg.Point(0, 0), cg.Point(4, 4)),
        cg.Segment(cg.Point(0, 4), cg.Point(4, 0)),
    ]
    rectangle = cg.Rectangle(cg.Point(-1, -1), cg.Point(5, 5))
    outer_ring = cg.LinearRing([cg.Point(0, 0), cg.Point(4, 0), cg.Point(4, 4), cg.Point(0, 4)])
    hole_ring = cg.LinearRing([cg.Point(1, 1), cg.Point(1, 3), cg.Point(3, 3), cg.Point(3, 1)])
    polygon = cg.Polygon(outer_ring, [hole_ring])

    document = cg.Document()
    document.add_layer("points", points=points)
    document.add_layer("segments", segments=diagonals)
    document.add_layer("rectangle", polygons=[rectangle.polygon()])
    document.add_layer("polygon_with_hole", polygons=[polygon])
    cg.save_yaml(document, str(output_path))

    print(f"Outer ring signed area: {outer_ring.signed_area()}")
    print(f"Hole ring signed area: {hole_ring.signed_area()}")
    print(f"Polygon area: {polygon.area()}")
    print(f"Rectangle contains polygon: {rectangle.contains(polygon)}")
    print(f"Rectangle contains center point: {rectangle.contains(cg.Point(2, 2))}")
    print(f"Wrote {output_path}")

    plot = cg.Plot("geometry primitives", "x", "y")
    plot.add_points(points, color="red", s=7)
    plot.add_segments(diagonals, color="gray", line_width=1, show_arrows=True)
    plot.add_polygon(rectangle.polygon(), face_color="lightgray", edge_color="black", alpha=0.15)
    plot.add_polygon(polygon, face_color="lightblue", edge_color="blue", alpha=0.35)
    plot.show()


if __name__ == "__main__":
    main()
