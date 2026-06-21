from pathlib import Path
from fractions import Fraction

import cgeom as cg


OUTPUT_DIR = Path("examples/output")
Coordinate = int | Fraction


def rectangle_segments(lower_left: tuple[Coordinate, Coordinate], upper_right: tuple[Coordinate, Coordinate]) -> list[cg.Segment]:
    rectangle = cg.Rectangle(cg.Point(*lower_left), cg.Point(*upper_right))
    return rectangle.ring().segments()


def rectangle_hole_segments(
    lower_left: tuple[Coordinate, Coordinate], upper_right: tuple[Coordinate, Coordinate]
) -> list[cg.Segment]:
    x0, y0 = lower_left
    x1, y1 = upper_right
    ring = cg.LinearRing(
        [
            cg.Point(x0, y0),
            cg.Point(x0, y1),
            cg.Point(x1, y1),
            cg.Point(x1, y0),
        ]
    )
    return ring.segments()


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "polygon_and_showcase.yaml"

    layer1_segments = rectangle_segments((1, 1), (7, 6)) + rectangle_hole_segments(
        (Fraction(11, 2), Fraction(9, 2)),
        (Fraction(13, 2), Fraction(11, 2)),
    )
    layer2_segments = rectangle_segments((4, 3), (10, 8)) + rectangle_hole_segments((5, 4), (6, 5))
    intersection_segments = cg.polygon_and(layer1_segments, layer2_segments)
    intersection_polygons = cg.assemble_polygons(intersection_segments)
    overlay_segments = sorted(cg.planarize_segments(layer1_segments + layer2_segments))

    document = cg.Document()
    document.add_layer("layer1", segments=layer1_segments)
    document.add_layer("layer2", segments=layer2_segments)
    document.add_layer("polygon_and_segments", segments=intersection_segments)
    document.add_layer("polygon_and_polygons", polygons=intersection_polygons)
    document.add_layer("overlay_segments", segments=overlay_segments)
    cg.save_yaml(document, str(output_path))

    print(f"Layer 1 segments: {len(layer1_segments)}")
    print(f"Layer 2 segments: {len(layer2_segments)}")
    print(f"Intersection segments: {len(intersection_segments)}")
    print(f"Intersection polygons: {len(intersection_polygons)}")
    print(f"Intersection area: {sum(polygon.area() for polygon in intersection_polygons)}")
    print(f"Wrote {output_path}")

    plot = cg.Plot("polygon and showcase", "x", "y")
    for polygon in intersection_polygons:
        plot.add_polygon(polygon, face_color="green", edge_color="darkgreen", alpha=0.45)
    plot.add_segments(layer1_segments, color="blue", line_width=4, show_arrows=True)
    plot.add_segments(layer2_segments, color="goldenrod", line_width=4, show_arrows=True)
    plot.show()


if __name__ == "__main__":
    main()
