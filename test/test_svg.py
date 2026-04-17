import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import math

import matplotlib.pyplot as plt
import shapely.plotting as spl
from matplotlib.font_manager import FontProperties
from matplotlib.path import Path
from matplotlib.textpath import TextPath
from shapely import Polygon
from shapely.geometry import LinearRing, MultiPolygon, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree
from svgelements import SVG, Path, Shape
from svgpathtools import svg2paths

from fieldpath.planner import FieldPathPlanner
import fieldpath.plot as fplt
fplt.set_plt_darkmode(plt)

from svgelements import SVG, Path, Shape, Move, Close
from shapely.geometry import Polygon, LinearRing, LineString
from shapely.ops import unary_union, polygonize

from svgelements import SVG, Path, Shape, Move, Close
from shapely.geometry import Polygon, LinearRing


def subpath_to_ring(segments, samples_per_segment=2):
    points = []
    first_point = None
    last_point = None

    for seg in segments:
        if first_point is None:
            first_point = seg.start

        # Segmentlänge, wenn vorhanden
        if hasattr(seg, "length"):
            seg_len = seg.length(error=1e-3)
        else:
            seg_len = 1.0

        n = max(2, int(seg_len) * samples_per_segment)

        for i in range(n + 1):
            t = i / n
            pt = seg.point(t)
            points.append((pt.x, pt.y))
            last_point = pt

    if not points:
        return None

    # schließen
    if first_point is not None and last_point is not None:
        if (first_point.x, first_point.y) != (last_point.x, last_point.y):
            points.append((first_point.x, first_point.y))

    ring = LinearRing(points)
    if ring.is_ring and ring.is_valid and ring.length > 0:
        return ring
    return None


def path_to_rings(path: Path, samples_per_segment=2):
    rings = []
    current_segments = []

    for seg in path:
        if isinstance(seg, Move):
            if current_segments:
                ring = subpath_to_ring(current_segments, samples_per_segment)
                if ring is not None:
                    rings.append(ring)
                current_segments = []
            continue
        if isinstance(seg, Close):
            if current_segments:
                ring = subpath_to_ring(current_segments, samples_per_segment)
                if ring is not None:
                    rings.append(ring)
                current_segments = []
            continue
        current_segments.append(seg)

    if current_segments:
        ring = subpath_to_ring(current_segments, samples_per_segment)
        if ring is not None:
            rings.append(ring)

    return rings


def svg_to_polygons_simple(svg_file, samples_per_segment=2):
    svg = SVG.parse(svg_file)

    polys = []

    for elem in svg.elements():
        # Nur Elemente, die sich in einen Pfad umwandeln lassen
        if not isinstance(elem, (Path, Shape)):
            continue

        style = getattr(elem, "values", {}) or {}
        fill = style.get("fill", elem.fill)

        # Nur explizit fill:none ignorieren
        if isinstance(fill, str) and fill.strip().lower() == "none":
            continue

        # In Path konvertieren
        path = elem if isinstance(elem, Path) else Path(elem)

        # Subpfade -> Ringe
        rings = path_to_rings(path, samples_per_segment=samples_per_segment)

        for r in rings:
            poly = Polygon(r)
            if poly.is_valid and not poly.is_empty and poly.area > 0:
                polys.append(poly)

    return polys




# Create Polygons from SVG Paths
polygons = svg_to_polygons_simple("docs/_static/logo_basic.svg", samples_per_segment=2)

# 2. Nach Fläche sortieren (größtes zuerst)
polys_sorted = sorted(polygons, key=lambda p: p.area, reverse=True)

result = []
used_as_hole = set()

for i, outer in enumerate(polys_sorted):
    if i in used_as_hole:
        continue

    current = outer
    for j, inner in enumerate(polys_sorted):
        if j == i or j in used_as_hole:
            continue
        # inner komplett in outer?
        if current.buffer(1e-9).contains(inner):
            # als Loch abziehen
            current = current.difference(inner)
            used_as_hole.add(j)

    # current kann MultiPolygon sein, wenn durch Differenzen zerschnitten
    if current.is_empty:
        continue
    if current.geom_type == "Polygon":
        result.append(current)
    elif current.geom_type == "MultiPolygon":
        result.extend(list(current.geoms))


fp = FieldPathPlanner(working_width=13.0, turn_radius=0.5)
plt.gca().invert_yaxis()
ax = plt.gca()
for i, poly in enumerate(result):
    print(f"Polygon {i}: Area={poly.area:.2f}, Holes={len(poly.interiors)}")
    working_path, _ = fp.generate_contoured_a_b_paths(poly, num_contour_lines=2, use_polygon_decomposition=True)
    path, line_types = fp.generate_work_sequence(working_path, start_pose=(0, 0, 0), field=poly)

    fplt.plot_field(poly, ax=ax)
    fplt.plot_working_path(path, ax=ax, types=line_types)
plt.show()

