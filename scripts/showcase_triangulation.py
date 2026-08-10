from pathlib import Path

import cgeom as cg


OUTPUT_DIR = Path("examples/output")


def vertex_classification_example() -> cg.LinearRing:
    return cg.LinearRing(
        [
            cg.Point(10, 5),
            cg.Point(8, 6),
            cg.Point(8, 11),
            cg.Point(6, 10),
            cg.Point(4, 12),
            cg.Point(0, 9),
            cg.Point(2, 7),
            cg.Point(1, 4),
            cg.Point(-1, 6),
            cg.Point(-2, 1),
            cg.Point(0, -2),
            cg.Point(3, -1),
            cg.Point(5, -2),
            cg.Point(4, 4),
            cg.Point(10, 2),
        ]
    )


def textbook_example() -> cg.LinearRing:
    return cg.LinearRing(
        [
            cg.Point(10, 8),  # v1
            cg.Point(7, 6),  # v2
            cg.Point(7, 12),  # v3
            cg.Point(5, 10),  # v4
            cg.Point(4, 13),  # v5
            cg.Point(0, 9),  # v6
            cg.Point(2, 7),  # v7
            cg.Point(1, 4),  # v8
            cg.Point(-1, 5),  # v9
            cg.Point(-2, 1),  # v10
            cg.Point(0, -2),  # v11
            cg.Point(3, -1),  # v12
            cg.Point(6, -5),  # v13
            cg.Point(5, 3),  # v14
            cg.Point(8, 0),  # v15
        ]
    )


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "triangulation_showcase.yaml"

    polygon = textbook_example()
    triangles = cg.triangulate(polygon)

    document = cg.Document()
    document.add_layer("input_polygon", polygons=[cg.Polygon(polygon)])
    document.add_layer("triangles", polygons=[cg.Polygon(ring) for ring in triangles])
    cg.save_yaml(document, str(output_path))

    print(f"Input vertices: {len(polygon.points)}")
    print(f"Triangles: {len(triangles)}")
    print(f"Wrote {output_path}")

    plot = cg.Plot("triangulation showcase", "x", "y")
    for triangle in triangles:
        plot.add_ring(triangle, edge_color="red", alpha=0.0)
    plot.add_ring(polygon)
    plot.show()


if __name__ == "__main__":
    main()
