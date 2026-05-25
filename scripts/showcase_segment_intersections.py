from pathlib import Path

import cgeom as cg


OUTPUT_DIR = Path("examples/output")
SEGMENT_COUNT = 1000
SEED = 0
ALGORITHM = "line-sweep"
MIN_LENGTH = 1
MAX_LENGTH = 40
MIN_COORD = 1
MAX_COORD = 1000


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    input_path = OUTPUT_DIR / "segment_intersections_input.yaml"
    output_path = OUTPUT_DIR / "segment_intersections_output.yaml"

    segments = cg.random_segments(
        count=SEGMENT_COUNT,
        seed=SEED,
        min_length=MIN_LENGTH,
        max_length=MAX_LENGTH,
        min_coord=MIN_COORD,
        max_coord=MAX_COORD,
    )

    intersections = cg.line_segment_intersection(segments, algorithm=ALGORITHM)

    input_document = cg.Document()
    input_document.add_layer("random_segments", segments=segments)
    cg.save_yaml(input_document, str(input_path))

    output_document = cg.Document()
    output_document.add_layer("random_segments", segments=segments)
    output_document.add_layer("intersections", points=intersections)
    cg.save_yaml(output_document, str(output_path))

    print(f"Segments: {len(segments)}")
    print(f"Intersections: {len(intersections)}")
    print(f"Algorithm: {ALGORITHM}")
    print(f"Wrote {input_path}")
    print(f"Wrote {output_path}")

    plot = cg.Plot("segment intersections", "x", "y")
    plot.add_segments(segments, color="gray", line_width=1, show_arrows=False)
    plot.add_points(intersections, color="red", s=5)
    plot.show()


if __name__ == "__main__":
    main()
