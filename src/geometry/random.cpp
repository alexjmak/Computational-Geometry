#include "geometry/random.hpp"
#include "algorithms/convex_hull.hpp"
#include "geometry/polygon.hpp"
#include <algorithm>

std::vector<Point> randomPoints(int num_points, int min_coordinate, int max_coordinate,
                                std::mt19937* rng) {
    if (!rng) {
        static std::mt19937 default_rng(std::random_device{}());
        rng = &default_rng;
    }
    std::uniform_int_distribution<int> dist(min_coordinate, max_coordinate);
    std::vector<Point> points;
    for (int i = 0; i < num_points; ++i) {
        points.emplace_back(dist(*rng), dist(*rng));
    }
    return points;
}

std::vector<Segment> randomSegments(int num_segments, int min_length, int max_length,
                                    int min_coordinate, int max_coordinate, std::mt19937* rng) {
    if (!rng) {
        static std::mt19937 default_rng(std::random_device{}());
        rng = &default_rng;
    }
    std::vector<Segment> segs;
    std::uniform_int_distribution<int> coord_dist(min_coordinate, max_coordinate);
    std::uniform_int_distribution<int> dir_dist(-max_length, max_length);
    int min_len2 = min_length * min_length;
    int max_len2 = max_length * max_length;

    while (segs.size() < num_segments) {
        int sx = coord_dist(*rng);
        int sy = coord_dist(*rng);

        // Sample integer direction with negatives + horizontal/vertical.
        int dx = dir_dist(*rng);
        int dy = dir_dist(*rng);
        if (dx == 0 && dy == 0)
            continue;
        int len2 = dx * dx + dy * dy;
        if (len2 < min_len2 || len2 > max_len2)
            continue;
        int ex = sx + dx;
        int ey = sy + dy;
        if (ex < min_coordinate || ex > max_coordinate || ey < min_coordinate ||
            ey > max_coordinate)
            continue;
        segs.emplace_back(Point(sx, sy), Point(ex, ey));
    }
    return segs;
}

Ring randomConvexPolygon(int n, int min_c, int max_c, std::mt19937* rng) {
    if (!rng) {
        static std::mt19937 default_rng(std::random_device{}());
        rng = &default_rng;
    }
    std::vector<Point> pts;
    for (int i = 0; i < std::max(3, n * 3); ++i) {
        pts.emplace_back((*rng)() % (max_c - min_c + 1) + min_c,
                         (*rng)() % (max_c - min_c + 1) + min_c);
    }
    return convexHull(pts);
}
