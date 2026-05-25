from pathlib import Path

import cgeom as cg


OUTPUT_DIR = Path("examples/output")
POINT_COUNT = 32
SEED = 7
MIN_COORD = -25
MAX_COORD = 25


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "showcase_convex_hull.yaml"

    points = cg.random_points(
        count=POINT_COUNT,
        seed=SEED,
        min_coord=MIN_COORD,
        max_coord=MAX_COORD,
    )
    hull = cg.convex_hull(points)
    hull_polygon = cg.Polygon(hull)

    document = cg.Document()
    document.add_layer("points", points=points)
    document.add_layer("convex_hull", polygons=[hull_polygon])
    cg.save_yaml(document, str(output_path))

    print(f"Input points: {len(points)}")
    print(f"Hull vertices: {len(hull.points)}")
    print(f"Hull area: {hull.area()}")
    print(f"Wrote {output_path}")

    plot = cg.Plot("convex hull", "x", "y")
    plot.add_points(points, color="gray", s=5)
    plot.add_ring(hull, face_color="lightblue", edge_color="blue", alpha=0.25)
    plot.add_segments(hull.segments(), color="blue", line_width=2, show_arrows=True)
    plot.show()


if __name__ == "__main__":
    main()
