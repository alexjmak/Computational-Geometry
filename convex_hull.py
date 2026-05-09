import time
from geometry import *
from plot import *

def slow_convex_hull(points: list[Point]) -> list[Segment]:
    """Build a convex hull by testing every candidate segment.

    Args:
        points: The input points to evaluate.

    Returns:
        A list of segments that lie on the convex hull.
    """
    segments = []
    for p in points:
        for q in points:
            if p == q: continue
            s = Segment(p, q)
            valid = True
            for r in points:
                if orientation(p, q, r) > 0: continue
                if is_point_on_segment(r, s): continue
                valid = False
                break
            if valid:
                segments.append(s)
                
    return segments


def build_half_hull(points) -> list[Point]:
    """Build one monotonic chain of the convex hull.

    Args:
        points: The points to process in sorted traversal order.

    Returns:
        The points that make up one half of the convex hull.
    """
    hull_points = []
    for point in points:
        hull_points.append(point)

        while len(hull_points) >= 3 and \
              orientation(hull_points[-3], hull_points[-2], hull_points[-1]) <= 0:
            hull_points.pop(-2)

    return hull_points


def convex_hull(points: list[Point]) -> Cycle:
    """Build the convex hull using the monotonic chain algorithm.

    Args:
        points: The input points to evaluate.

    Returns:
        A list of directed segments around the convex hull boundary.
    """
    sorted_points = sorted(points)
    lower_hull = build_half_hull(sorted_points)
    upper_hull = build_half_hull(reversed(sorted_points))
    return Cycle(lower_hull[:-1] + upper_hull[:-1])

def main():
    """Run a small convex hull example and display the result.

    Returns:
        None.
    """
    #points = [Point(Fraction(a.x, b.x), Fraction(a.y, b.y)) for a,b in zip(random_points(100), random_points(100, seed=2))]
    points = random_points(100)
    #print(points)
    #points = [Point(0, 0), Point(5, 0), Point(10, 0), Point(10, 10)]
    slow_start = time.perf_counter()
    slow_segments = slow_convex_hull(points)
    slow_elapsed = time.perf_counter() - slow_start

    fast_start = time.perf_counter()
    fast_hull = convex_hull(points)
    fast_elapsed = time.perf_counter() - fast_start

    print(f"slow_convex_hull: {slow_elapsed:.6f}s")
    print(f"convex_hull: {fast_elapsed:.6f}s")
    print(slow_segments)
    print(fast_hull)

    plot = Plot("Convex Hull")
    plot.add_points(points)
    plot.add_cycle(fast_hull)
    plt.show()

if __name__ == "__main__":
    main()
