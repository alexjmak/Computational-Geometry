import cgeom as cg

OUTPUT_PATH = "donut_island_assembled.yaml"


def main() -> None:
    segments = []
    r1 = cg.Rectangle(cg.Vect2(0, 0), cg.Vect2(10, 10))
    r2 = cg.Rectangle(cg.Vect2(3, 3), cg.Vect2(7, 7))
    r3 = cg.Rectangle(cg.Vect2(4, 4), cg.Vect2(6, 6))
    segments.extend(r1.cycle().segments())
    segments.extend(r2.cycle().segments())
    segments.extend(r3.cycle().segments())

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

    plot = cg.Plot(f"{OUTPUT_PATH}", "x", "y")
    for polygon in polygons:
        plot.add_polygon(polygon)
        for cycle in polygon.cycles():
            plot.add_segments(cycle.segments(), show_arrows=True)
    plot.show()

if __name__ == "__main__":
    main()
