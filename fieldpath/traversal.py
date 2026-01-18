from shapely import Polygon, LineString, Point, MultiPoint
from .utils import geom
from .utils.dubins_path import DubinsPath

class FieldTraversal:
    def __init__(self, fpp, field: Polygon = None):
        self.field = field
        self.fpp = fpp
        self.dubins = DubinsPath(fpp.turn_radius)

    def generate_path(self, start_pose=(0, 0, 0), end_pose=(0, 0, 0)) -> LineString:
            """ 
                Generate a traversal path from start_point to end_point that avoids exiting the field polygon.
                1) Check if direct line intersects field boundary
                2) If not, return direct line
                3) If yes, find contour lines that block the direct path
                4) For each blocking contour, find entry and exit points
                5) Navigate around the contour between entry and exit points (shorter path)

                :param start_point: Tuple of (x, y, angle)
                :param end_point: Tuple of (x, y, angle)
                :return: LineString with optimized path between the two points
            """
            # direct line from traversal start to traversal end
            direct_line = self.dubins.generate_path(start_pose, end_pose)

            if self.field is None:
                return direct_line
            
            # outline of the field, moved inside by half the working width
            # before we drive out the field boundary we drive along it.
            if self.fpp.traversal_offset is not None:
                offset_to_field_border = self.fpp.traversal_offset
            else:
                offset_to_field_border = self.fpp.working_width / 2 - 0.001  # small epsilon to avoid precision issues
            contour_drive_path, _ = self.fpp.generate_contour_paths(self.field, num_contour_lines=1, offset=offset_to_field_border)

            # Collect contours that block the direct path, along with their intersection points
            obstacles = []  # list of (contour, entry_point, exit_point)
            
            for contour in contour_drive_path.geoms:
                intersection = direct_line.intersection(contour)
                if intersection.is_empty: 
                    continue
                if isinstance(intersection, Point):
                    continue  # Single point = tangent, not blocking
                if isinstance(intersection, MultiPoint) and len(intersection.geoms) % 2 == 1:
                    continue  # Odd intersections = entering/leaving field boundary
                
                # Extract the intersection points
                intersect_pts = []
                if isinstance(intersection, MultiPoint):
                    intersect_pts = [Point(p) for p in intersection.geoms]
                elif hasattr(intersection, 'geoms'):
                    for g in intersection.geoms:
                        if isinstance(g, Point):
                            intersect_pts.append(g)
                        elif hasattr(g, 'coords'):
                            intersect_pts.extend([Point(c) for c in g.coords])
                elif hasattr(intersection, 'coords'):
                    intersect_pts = [Point(c) for c in intersection.coords]
                
                if len(intersect_pts) >= 2:
                    # Sort points by distance from start_point to get entry/exit in order
                    intersect_pts.sort(key=lambda p: Point(*start_pose[:2]).distance(p))
                    entry_pt = intersect_pts[0]
                    exit_pt = intersect_pts[-1]
                    obstacles.append((contour, entry_pt, exit_pt))
            
            # If no obstacles, return direct line
            if len(obstacles) == 0:
                return direct_line

            # Sort obstacles by distance from start to ensure correct order
            obstacles.sort(key=lambda x: Point(start_pose[:2]).distance(x[1]))

            # Build traversal path around each obstacle
            traversal_points = []
            last_pose = start_pose
            for contour, entry_pt, exit_pt in obstacles:
                # Navigate around the contour from entry to exit (shorter path)
                contour_segment = geom.extract_shorter_path(contour, entry_pt, exit_pt)
                contour_entry_pose = geom.pose_from_linestring(contour_segment, geom.LINE_POSITION_START)
                contour_exit_pose = geom.pose_from_linestring(contour_segment, geom.LINE_POSITION_END)
                # Generate Dubins path to contour entry
                to_entry_path = self.dubins.generate_path(last_pose, contour_entry_pose)
                # Add Dubins path (include first point only if this is the first segment)
                start_idx = 1 if len(traversal_points) > 0 else 0
                for coord in to_entry_path.coords[start_idx:]:
                    traversal_points.append(Point(coord))
                # Add contour segment (skip first point as it overlaps with Dubins endpoint)
                for coord in contour_segment.coords[1:]:
                    traversal_points.append(Point(coord))
                last_pose = contour_exit_pose

            # Finally, add direct segment from last exit to end_point
            to_end_path = self.dubins.generate_path(last_pose, end_pose)
            for coord in to_end_path.coords[1:]:
                traversal_points.append(Point(coord))

            return LineString([(pt.x, pt.y) for pt in traversal_points])