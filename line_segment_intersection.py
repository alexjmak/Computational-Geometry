import time
import math
from geometry import *
from functools import total_ordering

from geometry import Segment
from plot import *
from bintrees import AVLTree

debug = False
def brute_force_line_segment_intersection(segments: list[Segment]) -> set[Point]:
    """Return all pairwise intersection points by checking every segment pair.

    Args:
        segments: The segments to test for intersections.

    Returns:
        A set containing every unique intersection point found.
    """
    intersections = set()
    for s in segments:
        for t in segments:
            intersection = intersection_point(s, t)
            if intersection:
                intersections.add(intersection)
    return intersections

class LineSweep:
    def __init__(self):
        """Initialize sweep-line state and balanced trees for events and active segments."""
        self.k = 0
        self.curr_event = None
        self.event_queue = AVLTree()
        self.curr_segments = AVLTree()

    def populate_event_queue(self, segments : list[Segment]) -> None:
        """Seed the event queue with segment endpoints and their associated metadata.

        Args:
            segments: The segments whose endpoints should be inserted as events.
        """
        # Insert segment's upper endpoints into the event queue, and keep track of which segments they belong to
        for s in segments:
            lss = LineSweepSegment(s.canonicalized_y, self)
            upper_endpoint = LineSweepPoint(lss.segment.end)
            lower_endpoint = LineSweepPoint(lss.segment.start)
            if upper_endpoint not in self.event_queue:
                self.event_queue[upper_endpoint] = LineSweepEvent()
            self.event_queue[upper_endpoint].upper_segments.append(lss)
            if lower_endpoint not in self.event_queue:
                self.event_queue[lower_endpoint] = LineSweepEvent()
            self.event_queue[lower_endpoint].witnesses.add(lss)
        if not self.event_queue.is_empty():
            self.curr_event = self.event_queue.min_key()

    def print_element(self, key, value):
        """Print an active-segment entry with its current sweep-line intersection point.

        Args:
            key: The active line-sweep segment being printed.
            value: The AVL tree value associated with the key.
        """
        print(f"{key}: {intersect_at_y(key.segment, self.curr_event.point)}")

@total_ordering
class LineSweepSegment:
    def __init__(self, segment: Segment, line_sweep: LineSweep) -> None:
        """Wrap a segment with sweep-line-specific ordering and cached event data.

        Args:
            segment: The segment represented in the active set.
            line_sweep: The owning sweep-line instance.
        """
        self.segment = segment
        self.line_sweep = line_sweep
        dx = self.segment.start.x - self.segment.end.x
        dy = self.segment.start.y - self.segment.end.y
        if dy != 0:
            self._slope_inverse = Fraction(dx, -dy)
        else:
            self._slope_inverse = math.inf if dx >= 0 else -math.inf
        self._cached_event_x = None
        self._cached_event_y = None
        self._cached_point_at_event = None

    def point_at_curr_event(self) -> Optional[Point]:
        """Return this segment's intersection with the current sweep line.

        Returns:
            The point where the segment meets the horizontal line through the
            current event.
        """
        event_point = self.line_sweep.curr_event.point
        if self._cached_event_x == event_point.x and self._cached_event_y == event_point.y:
            return self._cached_point_at_event

        point_at_y = intersect_at_y(self.segment, event_point)
        self._cached_event_x = event_point.x
        self._cached_event_y = event_point.y
        self._cached_point_at_event = point_at_y
        return point_at_y

    def __eq__(self, other: LineSweepSegment) -> bool:
        """Check whether two sweep segments wrap the same segment.

        Args:
            other: The segment wrapper to compare against.

        Returns:
            True if both wrappers refer to the same segment and sweep instance,
            otherwise False.
        """
        return self.segment == other.segment and self.line_sweep == other.line_sweep

    def __lt__(self, other: LineSweepSegment) -> bool:
        """Order segments by their position relative to the current sweep event.

        Args:
            other: The segment wrapper to compare against.

        Returns:
            True if this segment should appear before the other in the active
            sweep ordering, otherwise False.
        """
        point_at_y = self.point_at_curr_event()
        other_point_at_y = other.point_at_curr_event()
        event_point = self.line_sweep.curr_event.point

        if point_at_y is None or other_point_at_y is None:
            raise AssertionError
        
        if point_at_y == other_point_at_y:
            slope_a = self._slope_inverse
            slope_b = other._slope_inverse

            if slope_a == slope_b:
                return self.segment < other.segment

            # Before and including the current event, the segments are ordered
            # in the order just below curr_event.point.y
            # If the intersection is after the current event, then it hasn't been
            # processed yet, they are ordered in the order just above curr_event.point.y
            if point_at_y.x <= event_point.x:
                return slope_a < slope_b
            else:
                return slope_a > slope_b

        return point_at_y < other_point_at_y

    def __str__(self) -> str:
        """Return the wrapped segment's string representation.

        Returns:
            The wrapped segment formatted as a string.
        """
        return str(self.segment)

    def __hash__(self):
        """Hash by the wrapped segment so instances can be stored in sets.

        Returns:
            A hash value derived from the wrapped segment.
        """
        return hash(self.segment)

@total_ordering
class LineSweepPoint:
    def __init__(self, point: Point) -> None:
        """Wrap a point with the event-queue ordering used by the sweep line.

        Args:
            point: The geometric point represented by this event key.
        """
        self.point = point

    def __eq__(self, other):
        """Check whether two sweep points refer to the same geometric point.

        Args:
            other: The point wrapper to compare against.

        Returns:
            True if both wrappers contain the same point, otherwise False.
        """
        return self.point == other.point

    def __lt__(self, other):
        """Order events from top to bottom, breaking ties from left to right.

        Args:
            other: The point wrapper to compare against.

        Returns:
            True if this point should be processed before the other event.
        """
        if self.point.y == other.point.y:
            return self.point.x < other.point.x
        return self.point.y > other.point.y

class LineSweepEvent:
    def __init__(self):
        """Store segments that start at an event point and segments witnessing that event."""
        self.upper_segments = []
        self.witnesses = set()

def find_new_event(predecessor: LineSweepSegment, successor: LineSweepSegment, ls_point: LineSweepPoint, line_sweep: LineSweep):
    """Insert a future intersection event for adjacent active segments when one exists.

    Args:
        predecessor: One active segment adjacent to the candidate event.
        successor: The other active segment adjacent to the candidate event.
        ls_point: The current event point.
        line_sweep: The sweep-line state to update.
    """
    intersect_point = intersection_point(predecessor.segment, successor.segment)
    if intersect_point is None:
        return
    ls_intersect_point = LineSweepPoint(intersect_point)

    if ls_intersect_point <= ls_point:
        return

    if ls_intersect_point in line_sweep.event_queue:
        event = line_sweep.event_queue[ls_intersect_point]
        event.witnesses.add(successor)
        event.witnesses.add(predecessor)
        return

    # Event queue only contains upper segments
    if debug: print(f"Found new event: {ls_intersect_point.point}")

    line_sweep.event_queue.insert(ls_intersect_point, LineSweepEvent())
    event = line_sweep.event_queue[ls_intersect_point]
    event.witnesses.add(successor)
    event.witnesses.add(predecessor)

def collect_touching_neighbors(ls_point: LineSweepPoint, seed: LineSweepSegment, mid_segments: list[LineSweepSegment], lower_segments: list[LineSweepSegment], neighbor_key_fn) -> None:
    """Walk neighboring active segments and collect those touching the current event point.

    Args:
        ls_point: The current event point.
        seed: A known active segment that touches the event point.
        mid_segments: Output list for segments containing the event in their interior.
        lower_segments: Output list for segments whose lower endpoint is the event.
        neighbor_key_fn: Function that returns the next neighboring active segment.
    """
    curr_segment = seed
    while True:
        try:
            curr_segment = neighbor_key_fn(curr_segment)
        except KeyError:
            break

        lower_endpoint = curr_segment.segment.start
        if lower_endpoint == ls_point.point:
            if debug: print(f"Found lower segment {curr_segment.segment}")
            lower_segments.append(curr_segment)
        elif is_point_on_segment(ls_point.point, curr_segment.segment):
            if debug: print(f"Found mid segment {curr_segment.segment}")
            mid_segments.append(curr_segment)
        else:
            break


def handle_event_point(ls_point: LineSweepPoint, event: LineSweepEvent, line_sweep: LineSweep):
    """Process one sweep event and report whether it represents an intersection.

    Args:
        ls_point: The event point being processed.
        event: The event data associated with the point.
        line_sweep: The sweep-line state to update.

    Returns:
        True if more than one segment meets at the event point, otherwise False.
    """
    upper_segments = event.upper_segments
    mid_segments = []
    lower_segments = []

    curr_segments = line_sweep.curr_segments
    if debug: curr_segments.foreach(line_sweep.print_element)

    # Populate all segments touching the event point on the lower point or anywhere in the middle
    # These are all adjacent in curr_segments, and we use a known probe segment to query the remaining ones
    probe_segment = next(iter(event.witnesses), None)
    if not probe_segment:
        dummy_segment = LineSweepSegment(Segment(ls_point.point, ls_point.point), line_sweep)
        prev_event = line_sweep.curr_event
        line_sweep.curr_event = ls_point

        try:
            curr_segment = curr_segments.floor_key(dummy_segment)
        except KeyError:
            pass
        else:
            if curr_segment.segment.start == ls_point.point or is_point_on_segment(ls_point.point, curr_segment.segment):
                probe_segment = curr_segment

        if not probe_segment:
            try:
                curr_segment = curr_segments.ceiling_key(dummy_segment)
            except KeyError:
                pass
            else:
                if curr_segment.segment.start == ls_point.point or is_point_on_segment(ls_point.point, curr_segment.segment):
                    probe_segment = curr_segment

        line_sweep.curr_event = prev_event

    if probe_segment:
        lower_endpoint = probe_segment.segment.start
        if lower_endpoint == ls_point.point:
            if debug: print(f"Found lower segment {probe_segment.segment}")
            lower_segments.append(probe_segment)
        elif is_point_on_segment(ls_point.point, probe_segment.segment):
            if debug: print(f"Found mid segment {probe_segment.segment}")
            mid_segments.append(probe_segment)

        collect_touching_neighbors(ls_point, probe_segment, mid_segments, lower_segments, curr_segments.succ_key)
        collect_touching_neighbors(ls_point, probe_segment, mid_segments, lower_segments, curr_segments.prev_key)

    for s in lower_segments:
        if debug: print(f"Remove lower {s.segment}")
        curr_segments.remove(s)

    for s in mid_segments:
        if debug: print(f"Remove mid {s.segment}")
        curr_segments.remove(s)

    # Move the sweep line
    line_sweep.curr_event = ls_point

    # Add in the upper segments and re-add the mid_segments so that they will be in the correct order in curr_segments
    upper_and_mid_segments = upper_segments + mid_segments
    for s in upper_and_mid_segments:
        if debug: print(f"Insert {s.segment}")
        curr_segments.insert(s, None)
    if debug: print()
    if debug: curr_segments.foreach(line_sweep.print_element)

    # Look for new events
    if len(upper_and_mid_segments) == 0:
        try:
            left_neighbor = curr_segments.floor_key(probe_segment)
            right_neighbor = curr_segments.ceiling_key(probe_segment)
        except KeyError:
            pass
        else:
            find_new_event(left_neighbor, right_neighbor, ls_point, line_sweep)
    else:
        leftmost = upper_and_mid_segments[0]
        rightmost = upper_and_mid_segments[0]
        for s in upper_and_mid_segments:
            leftmost = min(leftmost, s)
            rightmost = max(rightmost, s)

        try:
            left_neighbor = curr_segments.prev_key(leftmost)
        except KeyError:
            pass
        else:
            find_new_event(left_neighbor, leftmost, ls_point, line_sweep)

        try:
            right_neighbor = curr_segments.succ_key(rightmost)
        except KeyError:
            pass
        else:
            find_new_event(right_neighbor, rightmost, ls_point, line_sweep)


    if len(upper_segments) + len(mid_segments) + len(lower_segments) > 1:
        if debug: print(f"Intersection at {ls_point}")
        return True

    return False



def line_segment_intersection(segments: list[Segment]) -> set[Point]:
    """Find all segment intersection points using a Bentley-Ottmann-style sweep line.

    Args:
        segments: The segments to test for intersections.

    Returns:
        A set containing every unique intersection point found.
    """
    ls = LineSweep()
    ls.populate_event_queue(segments)

    intersection_points = set()
    while len(ls.event_queue) > 0:
        p, e = ls.event_queue.min_item()
        if debug: print()
        if debug: print("Event: " + str(p.point))
        ls.event_queue.remove(p)
        if handle_event_point(p, e, ls):
            intersection_points.add(p.point)

    return intersection_points

def main():
    """Run a simple benchmark and visualization for the line-segment intersection code."""
    #segments = []
    #segments.append(Segment(Point(0, 0), Point(0, 10)))
    #segments.append(Segment(Point(0, 5), Point(5, 5)))
    segments = [s for s in random_segments(1000, 1, 40, 1, 1000, rng=random.Random(2))]
    #segments = [Segment(a, b) for a,b in zip(random_points(200, rng=random.Random(0)), random_points(200, rng=random.Random(2)))]


    brute_force_start = time.perf_counter()
    brute_force_intersections = brute_force_line_segment_intersection(segments)
    brute_force_elapsed = time.perf_counter() - brute_force_start

    fast_start = time.perf_counter()
    intersections = line_segment_intersection(segments)

    if intersections == brute_force_intersections:
        print("Match")
    else:
        print("Mismatch")
    fast_elapsed = time.perf_counter() - fast_start

    print(f"brute force: {brute_force_elapsed:.6f}s")
    print(f"plane sweep: {fast_elapsed:.6f}s")
    plot = Plot("Line Segment Intersection")
    plot.add_segments(segments)
    plot.add_points(intersections)

    plt.show()

if __name__ == "__main__":
    main()
