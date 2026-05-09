from typing import Iterable

import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from geometry import *

class Plot:
    def __init__(self, title: str = None, x_label: str = "x", y_label: str = "y"):
        """Create a square Matplotlib figure configured for geometry plots.

        Args:
            title: The plot title.
            x_label: The x-axis label.
            y_label: The y-axis label.
        """
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_aspect("equal")
        self.ax.grid(True)
        self.ax.set_title(title)
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)

    def add_points(self, points: Iterable[Point], color: str = "red", s: int = 30) -> None:
        """Scatter plot a collection of points on the current axes.

        Args:
            points: The points to draw.
            color: The marker color.
            s: The marker size.
        """
        if len(points) == 0: return
        x_values = [point.x for point in points]
        y_values = [point.y for point in points]
        self.ax.scatter(x_values, y_values, color="red", s=s)

    def add_segments(self, segments: Iterable[Segment], color: str = "blue", line_width: int = 1) -> None:
        """Draw each segment in the iterable as a straight line on the axes.

        Args:
            segments: The segments to draw.
            color: The line color.
            line_width: The width of each plotted segment.
        """
        for segment in segments:
            self.ax.plot(
                [segment.start.x, segment.end.x],
                [segment.start.y, segment.end.y],
                color=color,
                linewidth=line_width,
            )

    def add_cycle(self, cycle: Cycle, face_color: str = "lightblue", edge_color: str = "blue", alpha: float = 0.35) -> None:
        """Fill a single closed cycle as a polygonal region.

        Args:
            cycle: The cycle to draw.
            face_color: The fill color.
            edge_color: The boundary color.
            alpha: The fill opacity.
        """
        if len(cycle.points) == 0:
            return

        x_values = [point.x for point in cycle.points]
        y_values = [point.y for point in cycle.points]
        self.ax.fill(x_values, y_values, facecolor=face_color, edgecolor=edge_color, alpha=alpha, linewidth=2)

    def add_polygon(self, polygon: Polygon, face_color: str = "lightblue", edge_color: str = "blue", alpha: float = 0.35) -> None:
        """Draw a polygon with one or more cycles, including holes when present.

        Args:
            polygon: The polygon to draw.
            face_color: The fill color.
            edge_color: The boundary color.
            alpha: The fill opacity.
        """
        vertices = []
        codes = []

        for cycle in polygon.cycles:
            if len(cycle.points) == 0:
                continue

            ring_points = list(cycle.points)

            ring = [(point.x, point.y) for point in ring_points]
            vertices.extend(ring)
            vertices.append(ring[0])
            codes.extend([Path.MOVETO] + [Path.LINETO] * (len(ring) - 1) + [Path.CLOSEPOLY])

        if len(vertices) == 0:
            return

        path = Path(vertices, codes)
        patch = PathPatch(path, facecolor=face_color, edgecolor=edge_color, alpha=alpha, linewidth=2)
        self.ax.add_patch(patch)


if __name__ == "__main__":
    outer = Cycle([
        Point(10, 10),
        Point(90, 10),
        Point(90, 90),
        Point(10, 90),
    ])
    inner_hole = Cycle([
        Point(30, 30),
        Point(30, 70),
        Point(70, 70),
        Point(70, 30),
    ])
    offset = 10
    outer2 = Cycle([
        Point(10 + offset, 10 + offset),
        Point(90 + offset, 10 + offset),
        Point(90 + offset, 90 + offset),
        Point(10 + offset, 90 + offset),
    ])
    inner_hole2 = Cycle([
        Point(30 + offset, 30 + offset),
        Point(30 + offset, 70 + offset),
        Point(70 + offset, 70 + offset),
        Point(70 + offset, 30 + offset),
    ])
    polygon = Polygon([outer, inner_hole])
    polygon2 = Polygon([outer2, inner_hole2])
    cycle = random_convex_polygon(2)

    plot = Plot("Polygon With Hole")
    plot.add_cycle(cycle)
    plot.add_points(cycle.points)
    #plot.add_polygon(polygon, face_color="lightgreen", edge_color="darkgreen", alpha=0.5)
    #plot.add_polygon(polygon2, face_color="lightgreen", edge_color="darkgreen", alpha=0.5)
    #plot.add_points(outer.points + inner_hole.points)
    plt.show()
