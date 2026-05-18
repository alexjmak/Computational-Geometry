import cgeom as cg

OUTPUT_PATH = "donut_island_assembled.yaml"


def rectangle_segments(lower_left: cg.Vect2, upper_right: cg.Vect2) -> list[cg.Segment]:
    lower_right = cg.Vect2(upper_right.x, lower_left.y)
    upper_left = cg.Vect2(lower_left.x, upper_right.y)
    return [
        cg.Segment(lower_left, lower_right),
        cg.Segment(lower_right, upper_right),
        cg.Segment(upper_right, upper_left),
        cg.Segment(upper_left, lower_left),
    ]


def main() -> None:
    segments = []
    segments.extend(rectangle_segments(cg.Vect2(0, 0), cg.Vect2(10, 10)))
    segments.extend(rectangle_segments(cg.Vect2(3, 3), cg.Vect2(7, 7)))
    segments.extend(rectangle_segments(cg.Vect2(4, 4), cg.Vect2(6, 6)))

    polygons = cg.assemble_polygons(segments)

    print(f"Assembled {len(polygons)} polygons")
    for i, polygon in enumerate(polygons):
        print(
            f"polygon {i}: outer_area={polygon.outer_cycle.area()}, "
            f"holes={len(polygon.inner_cycles)}, area={polygon.area()}"
        )

    document = cg.Document()
    document.add_layer("input_segments", segments=segments)
    document.add_layer("assembled_polygons", polygons=polygons)
    cg.save_yaml(document, OUTPUT_PATH)
    print(f"Wrote {OUTPUT_PATH}")

    cg.show(OUTPUT_PATH)


if __name__ == "__main__":
    main()
