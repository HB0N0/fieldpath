from __future__ import annotations

from typing import Tuple

import numpy as np
from shapely import (LineString, MultiLineString, MultiPoint, MultiPolygon,
                     Point, Polygon)
from shapely.affinity import rotate
from shapely.ops import nearest_points, split, substring

from .traversal import FieldTraversal
from .utils import geom
from .utils.convex_decomp import convex_decomp_sweep_line
from .utils.dubins_path import DubinsPath


class FieldPathPlanner:
    def __init__(self, working_width = 1/3, turn_radius = 0, traversal_offset = None):
        """New FieldPathPlanner instance.

        :param working_width: Width of the working tool (e.g., sprayer, seeder) in same units as field coordinates
        :param traversal_offset: Optional offset distance from field boundary when generating traversal paths
        """
        self.working_width = working_width
        self.traversal_offset = traversal_offset
        self.turn_radius = turn_radius

    def generate_A_B_line(self, field: Polygon, angle = 0.0) -> MultiLineString:
        """
        Generates a set of A-B lines covering the given polygonal field at the specified angle.
        
        :param self: Description
        :param field: field polygon with optional holes
        :type field: Polygon
        :param angle: Optional angle in degrees for line orientation (default is 0.0)
        """

        # First fix the transform origin to the center of the field polygon
        # The original polygon is rotated and intersected width the horizontal lines pattern
        transform_origin = [(field.bounds[0] + field.bounds[2]) / 2, 
                            (field.bounds[1] + field.bounds[3]) / 2]

        rotated_field = rotate(field, -angle, origin=transform_origin, use_radians=False)
        minx, miny, maxx, maxy = rotated_field.bounds

        width = maxx - minx
        height = maxy - miny
        diag = (width**2 + height**2) ** 0.5
        margin = diag  # overkill but safe

        line_minx = minx - margin
        line_maxx = maxx + margin

        # 4) Generate horizontal lines in the rotated system
        lines = []
        # Center the first line around miny to get nice symmetric coverage
        current_y = miny + self.working_width / 2 # start far below
        end_y = maxy + margin      # end far above

        while current_y <= end_y:
            line = LineString([(line_minx, current_y), (line_maxx, current_y)])
            # Intersect with the rotated polygon directly -> only keep the inside segments
            clipped = line.intersection(rotated_field)

            if not clipped.is_empty:
                if isinstance(clipped, LineString):
                    lines.append(LineString(clipped.coords))
                elif isinstance(clipped, MultiLineString):
                    # HIER: über .geoms iterieren, nicht direkt über clipped
                    for seg in clipped.geoms:
                        if not seg.is_empty:
                            lines.append(LineString(seg.coords))
            current_y += self.working_width

        if not lines:
            return MultiLineString([])
        # order lines, and reverse every second line for continuous path
        ordered_lines = []
        for i, line in enumerate(lines):
            if i % 2 == 0:
                ordered_lines.append(LineString(line.coords))
            else:
                ordered_lines.append(LineString(list(line.coords)[::-1]))
        result = MultiLineString(ordered_lines)

        # 5) Rotate back to original orientation
        result = rotate(result, angle, origin=transform_origin, use_radians=False)

        return result

    def generate_contour_paths(self, field: Polygon, num_contour_lines: int = 2, offset: float=None) -> Tuple[MultiLineString, Polygon]:
        """Generate contour lines inset from the field boundary.
        
        :param field: Polygon shape of a field
        :param num_contour_lines: Number of contour lines to generate, leave zero for no contours
        :param offset: Optional offset distance between contour lines, defaults to half working width
        :return: MultiLineString of contour lines, and the inner field polygon remaining after contours
        """
        if offset is None:
            offset = self.working_width / 2
        contours =  []
        for i in range(num_contour_lines):
            if i  == 0:
                contours.append(field.buffer(- offset))
            else:
                contours.append(contours[-1].buffer(- offset * 2))

        # The inner_field is the area not covered by contour lines
        if len(contours) > 0:
            inner_field = contours[-1].buffer(- offset)
        else:
            inner_field = field

        contour_lines = geom.unpack_boundary_lines(contours)
        return MultiLineString(contour_lines), inner_field
    
    def generate_contoured_a_b_paths(self, field: Polygon, num_contour_lines = 2, use_polygon_decomposition=False)-> MultiLineString:
        """
        Generate contour-following and A-B lines for the given field polygon.

        :param field: Polygon shape of a field
        :param num_contour_lines: Number of contour lines to generate, leave zero for no contours
        :param use_polygon_decomposition: If True, decompose inner field into convex sub-polygons for A-B line generation
        :return: MultiLineString of all generated lines, and list of sub-polygons if decomposition was used
        """
        # First generate contour lines (if any)
        contour_lines, inner_field = self.generate_contour_paths(field, num_contour_lines)
        all_lines = list(contour_lines.geoms)
        if not use_polygon_decomposition:
            # Get the dominant angle of the remaining inner field
            angle = geom.get_field_dominant_angle(inner_field)
            # Generate A-B lines for the inner field at the dominant angle
            lines = self.generate_A_B_line(inner_field, angle)

            all_lines.extend(lines.geoms)
            return MultiLineString(all_lines), None
        else:
            # Decompose inner field into convex sub-polygons
            # We set the sweep line angle 90 degress to the field orientation, to preserve the long parts first
            decomp_min_width = self.working_width
            decom_angle = geom.get_field_dominant_angle(inner_field) - 90
            sub_polygons = convex_decomp_sweep_line(inner_field, sweep_angle=decom_angle, min_width=decomp_min_width)
            for sub in sub_polygons:
                angle = geom.get_field_dominant_angle(sub)
                lines = self.generate_A_B_line(sub, angle)
                all_lines.extend(lines.geoms)
            return MultiLineString(all_lines), sub_polygons
    
    def generate_work_sequence(self, 
                               lines: MultiLineString, 
                               start_pose=(0, 0, 0), 
                               end_pose=None, 
                               field: Polygon=None,
                               contour_order='auto') -> Tuple[MultiLineString, list]:
        
        start_point = (start_pose[0], start_pose[1]) # Remove heading
        
        if not contour_order in ['first', 'last']:
            # If no order specified, order all geometries together
            ordered_lines = self._gnn_optimize_line_order(lines, start_point)
        else:
            # Separate contour and straight lines
            contour_lines, straight_lines = self._seperate_closed_open_lines(lines)
            if contour_order == 'first':
                ordered_contours = self._gnn_optimize_line_order(contour_lines, start_point)
                ordered_straights = self._gnn_optimize_line_order(straight_lines, 
                                                                  start_point=ordered_contours.geoms[-1].coords[-1] 
                                                                  if len(ordered_contours.geoms) > 0 else start_point)
                ordered_lines = MultiLineString(list(ordered_contours.geoms) + list(ordered_straights.geoms))
            else:  # contour_order == 'last'
                ordered_straights = self._gnn_optimize_line_order(straight_lines, start_point)
                ordered_contours = self._gnn_optimize_line_order(contour_lines, 
                                                                  start_point=ordered_straights.geoms[-1].coords[-1] 
                                                                  if len(ordered_straights.geoms) > 0 else start_point)
                ordered_lines = MultiLineString(list(ordered_straights.geoms) + list(ordered_contours.geoms))

        full_path, line_types = self._add_traversal_segments(ordered_lines, start_pose, end_pose, field)

        full_path, line_types = self._ensure_turn_radius(full_path, line_types, field)

        return full_path, line_types

    def _gnn_optimize_line_order(self, lines: MultiLineString, start_point=(0, 0)) -> MultiLineString:
        """ Optimize line order using a greedy nearest-neighbor algorithm.
            Each line can be traversed in either direction.
            Closed lines can start from any point on the contour.
            Heuristic: avoid crossing already driven lines when possible.
            Returns the optimized MultiLineString
        """
        if not lines or len(lines.geoms) == 0:
            return MultiLineString([])
        
        line_list = list(lines.geoms)
        unvisited = set(range(len(line_list)))
        optimized_lines = []
        driven_lines = []  # Track already driven lines for crossing detection
        current_pos = np.array(start_point)
        
        while unvisited:
            best_idx = None
            best_rev = False
            best_dist = float('inf')
            best_spun_line = None
            best_crossings = float('inf')  # Track crossing count
            
            for i in unvisited:
                coords = list(line_list[i].coords)
                start_pt = np.array(coords[0])
                end_pt = np.array(coords[-1])
                spun_line = None

                # If its a contour we can spin the starting point to the nearest point
                if line_list[i].is_closed:
                    spun_line = geom.spin_contour(line_list[i], current_pos)
                    coords = list(spun_line.coords)
                    start_pt = np.array(coords[0])
                    end_pt = np.array(coords[-1])
                
                # Check distance to both endpoints
                for reversed_flag in [False, True]:
                    entry_pt = end_pt if reversed_flag else start_pt
                    dist = np.linalg.norm(current_pos - entry_pt)
                    
                    # Count crossings with already driven lines
                    traversal = LineString([current_pos, entry_pt])
                    crossings = sum(1 for driven in driven_lines if traversal.intersects(driven))
                    
                    # Prefer fewer crossings; use distance as tiebreaker
                    if crossings < best_crossings or \
                       (crossings == best_crossings and dist < best_dist):
                        best_dist = dist
                        best_idx = i
                        best_rev = reversed_flag
                        best_spun_line = spun_line
                        best_crossings = crossings
            
            unvisited.remove(best_idx)
            # Use spun line if available, otherwise use original
            if best_spun_line is not None:
                coords = list(best_spun_line.coords)
            else:
                coords = list(line_list[best_idx].coords)
            
            if best_rev:
                coords = coords[::-1]
            
            line_obj = LineString(coords)
            optimized_lines.append(line_obj)
            driven_lines.append(line_obj)  # Track this line as driven
            current_pos = np.array(coords[-1])
        
        return MultiLineString(optimized_lines)
    
    def _add_traversal_segments(self, operation_lines: MultiLineString, start_point=None, end_point=None, field: Polygon=None) -> Tuple[MultiLineString, list]:
        """
        Add traversal segments between operation lines. 
        If field is provided, generate traversal paths that avoid exiting the field.
        :param operation_lines: MultiLineString of operation lines
        :param start_point: Optional starting point for the first traversal
        :param end_point: Optional end point for the last traversal
        :param field: Optional field polygon for traversal path generation
        :return: MultiLineString of full path including traversals, and list of line types
        """
        line_types = []
        return_path = []
        if len(operation_lines.geoms) == 0:
            return MultiLineString([]), line_types
        
        field_traversal = FieldTraversal(self, field)
        
        def generate_traversal(start, end):
            assert len(start) == 3 and len(end) == 3

            if(np.linalg.norm(np.array(start[:2]) - np.array(end[:2])) < 1e-6):
                return False
            traversal = field_traversal.generate_path(start, end)
            return_path.append(traversal)
            line_types.append('traversal')
            return True

        if start_point is not None:
            # add traversal from start_point to first line start
            first_line_pose = geom.pose_from_linestring(operation_lines.geoms[0], geom.LINE_POSITION_START)
            generate_traversal(start_point, first_line_pose)
        for i, line in enumerate(operation_lines.geoms):
            if i > 0:
                prev_line = operation_lines.geoms[i - 1]
                line_end_pose = geom.pose_from_linestring(prev_line, geom.LINE_POSITION_END)
                line_start_pose = geom.pose_from_linestring(line, geom.LINE_POSITION_START)
                generate_traversal(line_end_pose, line_start_pose)
            return_path.append(line)
            line_types.append('operation')

        if end_point is not None:
            # add traversal from last line end to end_point
            last_line_pose = geom.pose_from_linestring(operation_lines.geoms[-1], geom.LINE_POSITION_END)
            generate_traversal(last_line_pose, end_point)

        return MultiLineString(return_path), line_types
    
    
    def _seperate_closed_open_lines(self, lines: MultiLineString) -> Tuple[MultiLineString, MultiLineString]:
        """ Seperates closed contours from open lines in a MultiLineString """
        closed_lines = []
        open_lines = []
        for line in lines.geoms:
            if line.is_closed:
                closed_lines.append(line)
            else:
                open_lines.append(line)
        return MultiLineString(closed_lines), MultiLineString(open_lines)
    
    def _ensure_turn_radius(self, lines: MultiLineString, line_types: list, field: Polygon=None) -> Tuple[MultiLineString, list]:
        """ Checks angle between consecutive waypoints and adds Dubins paths if needed to respect turn radius.
            Returns new MultiLineString and updated line_types list.
        """
        if self.turn_radius <= 0:
            return lines, line_types  # No turn radius constraint

        new_lines = []
        new_line_types = []
        # TODO: implement turn with check for obstacles. for now disabled. (= No field defined)
        field_traversal = FieldTraversal(self, field=None)
        
        for i in range(len(lines.geoms)):
            current_line = lines.geoms[i]
            
            line_segments = geom.split_line_at_sharp_angles(current_line, angle_threshold=135)
            self.corner_method = 'fillet'  # 'fillet'  # 'dubins'  #
            if self.corner_method == 'dubins':
                new_lines.append(line_segments[0])
                new_line_types.append(line_types[i])
                if len(line_segments) > 1:
                    for seg in range(1,len(line_segments)):
                        # Add traversal path at sharp angles
                        dubins_path = field_traversal.generate_path(
                            geom.pose_from_linestring(line_segments[seg - 1], geom.LINE_POSITION_END),
                            geom.pose_from_linestring(line_segments[seg], geom.LINE_POSITION_START)
                        )
                        new_lines.append(dubins_path)
                        new_line_types.append('traversal')
                        
                        new_lines.append(line_segments[seg])
                        new_line_types.append(line_types[i])
            elif self.corner_method == 'fillet':
                new_lines.append(line_segments[0])
                new_line_types.append(line_types[i])
                if len(line_segments) > 1:
                    for seg in range(len(line_segments) - 1):
                        start_line = new_lines[-1]
                        end_line = line_segments[seg + 1]
                        line_before, fillet, line_after = geom.add_fillet(
                            start_line, end_line, self.turn_radius
                        )
                        new_lines[-1] = line_before  # replace last line with modified line_before
                        new_line_types[-1] = line_types[i]
                        new_lines.append(fillet)
                        new_line_types.append('traversal')
                        new_lines.append(line_after)
                        new_line_types.append(line_types[i])
                
            else:
                new_lines.append(current_line)
                new_line_types.append(line_types[i])


        return MultiLineString(new_lines), new_line_types
        
    