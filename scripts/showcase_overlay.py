from pathlib import Path

import cgeom as cg


OUTPUT_DIR = Path("examples/output")


def point(x: int, y: int) -> cg.Point:
    return cg.Point(x, y)


def segment(a: tuple[int, int], b: tuple[int, int]) -> cg.Segment:
    return cg.Segment(point(*a), point(*b))


def segments_from_edges(edges: list[tuple[tuple[int, int], tuple[int, int]]]) -> list[cg.Segment]:
    return [segment(a, b) for a, b in edges]


def solid_layer() -> list[cg.Segment]:
    vertices = [
        (20, 72),
        (82, 71),
        (92, 46),
        (74, 21),
        (26, 28),
        (13, 57),
    ]
    edges = list(zip(vertices, vertices[1:] + vertices[:1]))
    edges.extend(
        [
            ((20, 72), (35, 62)),
            ((35, 62), (60, 60)),
            ((60, 60), (82, 71)),
            ((35, 62), (40, 38)),
            ((40, 38), (64, 45)),
            ((60, 60), (64, 45)),
            ((64, 45), (82, 48)),
            ((74, 21), (82, 48)),
            ((26, 28), (40, 38)),
        ]
    )
    return segments_from_edges(edges)


def dotted_layer() -> list[cg.Segment]:
    vertices = [
        (3, 70),
        (42, 86),
        (74, 58),
        (70, 43),
        (28, 9),
        (3, 35),
    ]
    edges = list(zip(vertices, vertices[1:] + vertices[:1]))
    edges.extend(
        [
            ((9, 60), (38, 65)),
            ((38, 65), (52, 53)),
            ((52, 53), (47, 44)),
            ((47, 44), (11, 44)),
            ((11, 44), (9, 60)),
        ]
    )
    return segments_from_edges(edges)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "overlay_showcase.yaml"

    layer1_segments = solid_layer()
    layer2_segments = dotted_layer()
    overlay = cg.segment_overlay(layer1_segments, layer2_segments)
    overlay_faces = overlay.polygons()
    overlay_segments = sorted(cg.planarize_segments(layer1_segments + layer2_segments))
    intersections = cg.line_segment_intersection(layer1_segments + layer2_segments)

    left_only_polygons = []
    right_only_polygons = []
    both_polygons = []
    for face in overlay_faces:
        in_left = face.label.left_face != 0
        in_right = face.label.right_face != 0
        if in_left and in_right:
            both_polygons.append(face.polygon)
        elif in_left:
            left_only_polygons.append(face.polygon)
        elif in_right:
            right_only_polygons.append(face.polygon)

    document = cg.Document()
    document.add_layer("layer1_solid", segments=layer1_segments)
    document.add_layer("layer2_dotted", segments=layer2_segments)
    document.add_layer("overlay_left_only", polygons=left_only_polygons)
    document.add_layer("overlay_right_only", polygons=right_only_polygons)
    document.add_layer("overlay_both", polygons=both_polygons)
    document.add_layer("overlay_segments", segments=overlay_segments)
    cg.save_yaml(document, str(output_path))

    print(f"Layer 1 segments: {len(layer1_segments)}")
    print(f"Layer 2 segments: {len(layer2_segments)}")
    print(f"Overlay faces: {len(overlay_faces)}")
    print(f"Overlay segments: {len(overlay_segments)}")
    print(f"Wrote {output_path}")

    plot = cg.Plot("overlay showcase", "x", "y")
    for polygon in left_only_polygons:
        plot.add_polygon(polygon, face_color="yellow", edge_color="goldenrod", alpha=0.45)
    for polygon in right_only_polygons:
        plot.add_polygon(polygon, face_color="blue", edge_color="navy", alpha=0.25)
    for polygon in both_polygons:
        plot.add_polygon(polygon, face_color="green", edge_color="darkgreen", alpha=0.45)
    plot.add_segments(layer1_segments, color="goldenrod", line_width=4, show_arrows=False)
    plot.add_segments(layer2_segments, color="navy", line_width=4, show_arrows=False)
    plot.add_points(intersections, color="black", s=10)
    plot.show()


if __name__ == "__main__":
    main()
