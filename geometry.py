import math
import random
from enum import Enum
from fractions import Fraction
from functools import total_ordering
from typing import Optional


@total_ordering
class Vect2:
    def __init__(self, x: int | Fraction, y: int | Fraction) -> None:
        """Initialize a 2D vector.

        Args:
            x: The x coordinate.
            y: The y coordinate.
        """
        if not isinstance(x, (int, Fraction)) or not isinstance(y, (int, Fraction)):
            raise TypeError("Coordinates must be int or Fraction")

        self.x = x
        self.y = y

    def __eq__(self, other: Vect2) -> bool:
        if not isinstance(other, Vect2):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __add__(self, other: Vect2) -> Vect2:
        if not isinstance(other, Vect2):
            return NotImplemented
        return Vect2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vect2) -> Vect2:
        if not isinstance(other, Vect2):
            return NotImplemented
        return Vect2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: int | Fraction) -> Vect2:
        if isinstance(scalar, int) or isinstance(scalar, Fraction) :
            return Vect2(self.x * scalar, self.y * scalar)
        return NotImplemented

    __rmul__ = __mul__

    def __neg__(self) -> Vect2:
        return Vect2(-self.x, -self.y)

    def __lt__(self, other: Vect2) -> bool:
        if not isinstance(other, Vect2):
            return NotImplemented
        if self.x == other.x:
            return self.y < other.y
        return self.x < other.x

    def __str__(self) -> str:
        return f"({float(self.x)}, {float(self.y)})"

    __repr__ = __str__

    def __hash__(self):
        return hash((self.x, self.y))

Point = Vect2

@total_ordering
class Segment:
    def __init__(self, start: Point, end: Point) -> None:
        """Initialize a directed segment.

        Args:
            start: The starting endpoint.
            end: The ending endpoint.
        """
        self.start = start
        self.end = end

    @property
    def is_canonicalized_x(self) -> bool:
        """Check whether the segment endpoints are in lexicographic order.

        Returns:
            True if start is less than or equal to end, otherwise False.
        """
        return self.start <= self.end

    @property
    def is_canonicalized_y(self) -> bool:
        if self.start.y == self.end.y:
            return self.start >= self.end
        return self.start.y < self.end.y

    @property
    def canonicalized_x(self) -> Segment:
        """Return a copy of the segment with lexicographically ordered endpoints.

        Returns:
            A segment whose start endpoint is less than or equal to its end
            endpoint.
        """
        if self.is_canonicalized_x:
            return Segment(self.start, self.end)
        return Segment(self.end, self.start)

    @property
    def canonicalized_y(self) -> Segment:
        if self.is_canonicalized_y:
            return Segment(self.start, self.end)
        return Segment(self.end, self.start)

    def __eq__(self, other: Segment) -> bool:
        if not isinstance(other, Segment):
            return NotImplemented
        return self.start == other.start and self.end == other.end

    def __lt__(self, other):
        if not isinstance(other, Segment):
            return NotImplemented
        if self.start == other.start:
            return self.end < other.end
        return self.start < other.start

    def __str__(self) -> str:
        return f"S({self.start}, {self.end})"

    __repr__ = __str__

    def __hash__(self):
        return hash((self.start, self.end))

class Cycle:
    def __init__(self, points: list[Point]) -> None:
        """Initialize a polygonal cycle from an ordered list of points.

        Args:
            points: The points that define the closed cycle boundary.
        """
        self.points = points

    @property
    def signed_area(self) -> float:
        """Compute the signed area of the cycle.

        Returns:
            The signed area of the cycle. A positive value indicates
            counter-clockwise point order, and a negative value indicates
            clockwise point order.
        """
        if len(self.points) < 3:
            return 0.0
        ret = 0.0
        prev = self.points[-1]
        for point in self.points:
            ret += cross_product(prev, point)
            prev = point
        return 0.5 * ret

    @property
    def area(self) -> float:
        """Compute the unsigned area of the cycle.

        Returns:
            The absolute area enclosed by the cycle.
        """
        return abs(self.signed_area)

    @property
    def is_outer(self) -> bool:
        """Check whether the cycle is oriented counter-clockwise.

        Returns:
            True if the cycle has positive signed area, otherwise False.
        """
        return self.signed_area > 0.0

    def segments(self) -> list[Segment]:
        """Convert the cycle into its directed boundary segments.

        Returns:
            A list of segments connecting consecutive points in the cycle.
        """
        segments = []
        prev = self.points[-1]
        for point in self.points:
            segments.append(Segment(prev, point))
            prev = point
        return segments

class Polygon:
    def __init__(self, cycles: list[Cycle]) -> None:
        """Initialize a polygon from one or more boundary cycles.

        Args:
            cycles: The boundary cycles that make up the polygon.
        """
        self.cycles = cycles

    @property
    def area(self) -> float:
        """Compute the signed area contributed by all polygon cycles.

        Returns:
            The sum of the signed areas of the polygon's cycles.
        """
        ret = 0.0
        for cycle in self.cycles:
            ret += cycle.signed_area
        return ret

def random_points(num_points: int, min_coordinate: int = 10, max_coordinate: int = 100, rng: random.Random=random.Random()) -> list[Point]:
    """Generate a repeatable set of random 2D points.

    Args:
        num_points: The number of points to generate.
        min_coordinate: The minimum x and y coordinate value.
        max_coordinate: The maximum x and y coordinate value.
        seed: The random seed used to make generation repeatable.

    Returns:
        A list of randomly generated points.
    """

    return [
        Point(
            rng.randint(min_coordinate, max_coordinate),
            rng.randint(min_coordinate, max_coordinate),
        )
        for _ in range(num_points)
    ]

def random_segments(num_segments: int, min_length: int = 1, max_length = 10, min_coordinate: int = 10, max_coordinate: int = 100, rng: random.Random=random.Random()) -> list[Segment]:
    segs = []

    min_len2 = min_length * min_length
    max_len2 = max_length * max_length

    while len(segs) < num_segments:
        sx = rng.randint(min_coordinate, max_coordinate)
        sy = rng.randint(min_coordinate, max_coordinate)

        # sample integer direction with negatives + horizontal/vertical
        dx = rng.randint(-max_length, max_length)
        dy = rng.randint(-max_length, max_length)
        if dx == 0 and dy == 0:
            continue
        len2 = dx * dx + dy * dy
        if not (min_len2 <= len2 <= max_len2):
            continue

        ex = sx + dx
        ey = sy + dy
        if not (min_coordinate <= ex <= max_coordinate and min_coordinate <= ey <= max_coordinate):
            continue

        segs.append(Segment(Point(sx, sy), Point(ex, ey)))

    return segs

def random_convex_polygon(n: int, min_c=10, max_c=100, rng: random.Random=random.Random()) -> Cycle:
    from convex_hull import convex_hull
    pts = [Point(rng.randint(min_c, max_c), rng.randint(min_c, max_c)) for _ in range(max(3, n*3))]
    hull = convex_hull(pts)
    return hull

def cross_product(p: Vect2, q: Vect2) -> int:
    """Compute the 2D cross product of two vectors.

    Args:
        p: The first vector.
        q: The second vector.

    Returns:
        The scalar cross product p x q.
    """
    return p.x * q.y - p.y * q.x

def dot_product(p: Vect2, q: Vect2) -> int:
    """Compute the 2D dot product of two vectors represented by Vect2.

    Args:
        p: The first vector.
        q: The second vector.

    Returns:
        The scalar dot product p x q.
    """
    return p.x * q.x + p.y * q.y

def orientation(p: Point, q: Point, r: Point) -> int:
    """Determine the orientation of three points.

    Args:
        p: The first point.
        q: The second point.
        r: The third point.

    Returns:
        A positive value for counter-clockwise, a negative value for clockwise,
        or zero if the points are collinear.
    """
    return cross_product(q - p, r - p)

def is_point_on_segment(p: Point, s: Segment) -> bool:
    """Check whether a point lies on a segment.

    Args:
        p: The point to test.
        s: The segment to test against.

    Returns:
        True if the Vect2 is on the segment, otherwise False.
    """
    is_collinear = orientation(s.start, s.end, p) == 0
    if not is_collinear:
        return False

    # Vectors from the segment endpoints to the point are in opposite directions
    return dot_product(p - s.end, p - s.start) <= 0

class IntersectionType(Enum):
    """Classify how two segments intersect."""

    NONE = 0
    POINT = 1
    SEGMENT = 2

def intersection_type(s: Segment, t: Segment) -> IntersectionType:
    """Return the number of intersection points between two segments.

    Args:
        s: The first segment.
        t: The second segment.

    Returns:
        IntersectionType.NONE if the segments do not intersect.
        IntersectionType.POINT if the segments intersect at exactly one point.
        IntersectionType.SEGMENT if the segments overlap along a non-zero-length
        collinear segment.

    Notes:
        Endpoint touching and T-intersections are classified as
        IntersectionType.POINT. Collinear segments that overlap only at a single
        endpoint are also classified as IntersectionType.POINT, not
        IntersectionType.SEGMENT.
    """
    o1 = orientation(s.start, s.end, t.start)
    o2 = orientation(s.start, s.end, t.end)
    o3 = orientation(t.start, t.end, s.start)
    o4 = orientation(t.start, t.end, s.end)

    # Coincident segments
    if o1 == 0 and o2 == 0 and o3 == 0 and o4 == 0:
        s2 = s.canonicalized_x
        t2 = t.canonicalized_x
        if s2.end > t2.start and s2.start < t2.end:
            return IntersectionType.SEGMENT

    # X intersection
    if o1 * o2 < 0 and o3 * o4 < 0:
        return IntersectionType.POINT

    # T intersection
    if o1 == 0 and is_point_on_segment(t.start, s):
        return IntersectionType.POINT
    if o2 == 0 and is_point_on_segment(t.end, s):
        return IntersectionType.POINT
    if o3 == 0 and is_point_on_segment(s.start, t):
        return IntersectionType.POINT
    if o4 == 0 and is_point_on_segment(s.end, t):
        return IntersectionType.POINT

    return IntersectionType.NONE



def intersection_point(s: Segment, t: Segment) -> Optional[Point]:
    """Compute the intersection point of two segments when it is unique.

    Args:
        s: The first segment.
        t: The second segment.

    Returns:
        The unique intersection point if one exists. Returns None if the
        segments do not intersect or if they overlap along a segment.
    """

    if intersection_type(s, t) != IntersectionType.POINT:
        return None

    """
    Parametric form of the two lines:
        p = s.start + a * (s.end - s.start)
        p = t.start + b * (t.end - t.start)
    
    Rearranged into a 2x2 system:
        a * (s.end - s.start) + b * (t.start - t.end) = t.start - s.start
    """
    det = cross_product(s.end - s.start, t.start - t.end)

    # Collinear endpoints touching
    if det == 0:
        if is_point_on_segment(t.start, s):
            return t.start
        if is_point_on_segment(t.end, s):
            return t.end
        if is_point_on_segment(s.start, t):
            return s.start
        if is_point_on_segment(s.end, t):
            return s.end

    # Cramer's Rule
    det_a = cross_product(t.start - s.start, t.start - t.end)
    a = Fraction(det_a, det)

    return s.start + (s.end - s.start) * a

def intersect_at_y(s: Segment, p: Point) -> Optional[Point]:
    cs = s.canonicalized_y

    if p.y < cs.start.y or p.y > cs.end.y:
        return None

    if cs.start.y == cs.end.y:
        if p.x > cs.start.x or p.x < cs.end.x:
            return cs.start
        return p

    t = Fraction(p.y - cs.start.y, cs.end.y - cs.start.y)
    return cs.start + (cs.end - cs.start) * t