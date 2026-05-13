import cgeom as cg
import fractions

RANDOM_PATH = "random.yaml"
OUTPUT_PATH = "output.yaml"


def main() -> None:
    segments = cg.random_segments(count=10000, seed=0)

    random_document = cg.Document()
    random_document.add_layer("random_segments", segments=segments)
    cg.save_yaml(random_document, RANDOM_PATH)

    intersections = cg.line_segment_intersection(segments)

    output_document = cg.load_yaml(RANDOM_PATH)
    output_document.add_layer("intersections", points=intersections)
    cg.save_yaml(output_document, OUTPUT_PATH)

    print(f"Wrote {RANDOM_PATH}")
    print(f"Found {len(intersections)} intersections")
    print("Algorithm: line-sweep")
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
