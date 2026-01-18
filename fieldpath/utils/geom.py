from typing import Tuple
from shapely import Polygon, LineString, Point, MultiLineString
from shapely.ops import substring, nearest_points
import numpy as np

LINE_POSITION_END = -1
LINE_POSITION_START = 0

def pose_from_linestring(line: LineString, index: int):
    """
    Get position (x, y, heading) from LineString at given index. To get the end of a line use index -1.

    :param line: LineString where the position and heading should be extracted
    :param index: Index at which point the heading and position should be extracted
    :return: Tuple(x, y, heading)
    """
    coords = list(line.coords)
    if len(coords) < 2:
        x, y = coords[0]
        return (x, y, 0.0)
        
    if index == -1:
        x, y = coords[-1]
        x_prev, y_prev = coords[-2]
        heading = np.degrees(np.arctan2(y - y_prev, x - x_prev))
        return (x, y, heading)
    elif 0 <= index < len(coords):
        x, y = coords[index]
        if index < len(coords) - 1:
            x_next, y_next = coords[index + 1]
        else:
            x_next, y_next = coords[index - 1]
        heading = np.degrees(np.arctan2(y_next - y, x_next - x))
        return (x, y, heading)
    return None

def extract_shorter_path(line: LineString, start_point: Point, end_point: Point) -> LineString:
    """
    Return the shorter path along a closed LineString from start to end.

    :param line: Closed contour
    :param start_point: point on the contour line
    :param end_point: point on the contour line
    :return: shortest segment from start to end, following the contour line
    """
    start = line.project(start_point)
    end = line.project(end_point)

    # Calculate distances for both directions
    if end >= start:
        forward_len = end - start
        backward_len = line.length - forward_len
    else:
        forward_len = (line.length - start) + end
        backward_len = start - end

    if forward_len <= backward_len:
        # Go forward along the line
        if end >= start:
            return substring(line, start, end)
        else:
            # Wrap forward: From start to line end, then 0 to end
            part1 = substring(line, start, line.length)
            part2 = substring(line, 0, end)
            return LineString(list(part1.coords) + list(part2.coords)[1:])
    else:
        # Go backward along the line
        if start >= end:
            # From end to start (reverse order)
            part1 = substring(line, end, start)
            return LineString(list(part1.coords)[::-1])
        else:
            # Wrap backward: From start to 0, then end to line end
            part1 = substring(line, 0, start)
            part2 = substring(line, end, line.length)
            coords = list(part1.coords)[::-1] + list(part2.coords)[::-1][1:]
            return LineString(coords)

def unpack_boundary_lines(obj_list: list) -> MultiLineString:
    """
    Unpack all geometries from a list of Polygon or MultiPolygon objects and append them to a MultiLineString
    """
    boundary_lines = []
    # unpack contour boundaries into LineStrings (some field geometries may have multiple contour parts (holes,..))
    for c in obj_list:
        if c.is_empty:
            continue
        boundary = c.boundary
        if isinstance(boundary, LineString):
            boundary_lines.append(LineString(boundary.coords))
        elif isinstance(boundary, MultiLineString):
            for g in boundary.geoms:
                boundary_lines.append(LineString(g.coords))
        else:
            # in case of GeometryCollection, etc.
            for g in getattr(boundary, "geoms", []):
                if isinstance(g, LineString) and not g.is_empty:
                    boundary_lines.append(LineString(g.coords))
    return boundary_lines

def spin_contour(contour: LineString, start_point: Tuple[float, float]) -> LineString:
    """
    Rotate closed contour LineString so that it starts closest to start_point
    """
    if not contour.is_closed:
        return contour  # only closed contours can be spun

    start_pt = Point(start_point)
    nearest_pt = nearest_points(start_pt, contour)[1]
    proj_dist = contour.project(nearest_pt)

    # Create new LineString starting from nearest_pt
    part1 = substring(contour, proj_dist, contour.length)
    part2 = substring(contour, 0, proj_dist)
    spun_coords = list(part1.coords) + list(part2.coords)[1:]  # avoid duplicate point
    return LineString(spun_coords)

def get_field_dominant_angle(field: Polygon) -> float:
    """
    Returns the orientation of the minimum rotated rectangle
    of the polygon. which can be used as estimate for a good driving direction.

    :param field: Polygon shape of a field
    :return: Angle in degrees from 0 to 180
    """
    # Get the points of the minimum rotated rectangle around the field
    mrr = field.minimum_rotated_rectangle
    coords = np.asarray(mrr.exterior.coords[:3])  # first 3 points
    if len(coords) < 3:
        return 0.0  # If not enough points, return 0

    # Two edge vectors
    v1 = coords[1] - coords[0]
    v2 = coords[2] - coords[1]

    # Pick the longer edge
    lengths = np.linalg.norm([v1, v2], axis=1)
    # vector x and y components of longest edge
    dx, dy = (v1, v2)[np.argmax(lengths)]

    # Angle from x-axis in degrees, folded to [0, 180)
    angle = np.degrees(np.arctan2(dy, dx)) % 180
    return float(angle)

def calculate_angle(p1: Tuple[float, float], p2: Tuple[float, float], p3: Tuple[float, float]) -> float:
    """
    Calculate the angle between p2->p1 and p2->p3 in degrees.
    
    :param p1: Direction 1
    :type p1: Tuple[float, float]
    :param p2: Origin point
    :type p2: Tuple[float, float]
    :param p3: Direction 2
    :type p3: Tuple[float, float]
    :return: angle in degrees
    """
    p2_p1 = np.array(p1) - np.array(p2)
    p2_p3 = np.array(p3) - np.array(p2)

    cosine_angle = np.dot(p2_p1, p2_p3) / (np.linalg.norm(p2_p1) * np.linalg.norm(p2_p3))
    angle = np.arccos(cosine_angle)
    return np.degrees(angle)    

def split_line_at_sharp_angles(line: LineString, angle_threshold=45):
    """
    Check a linestring for sharp angles and cut it at these points.
    
    Parameters:
    - line: shapely LineString
    - angle_threshold: angles below this value (in degrees) are considered sharp
    
    Returns:
    - list of LineString segments
    """
    coords = list(line.coords)
    
    if len(coords) < 3:
        return [line]
    
    segments = []
    current_segment = [coords[0]]
    
    for i in range(1, len(coords) - 1):
        current_segment.append(coords[i])
        
        # Calculate angle at this point
        angle = calculate_angle(coords[i-1], coords[i], coords[i+1])
        
        # If sharp angle detected, split here
        if angle < angle_threshold:
            if len(current_segment) >= 2:
                segments.append(LineString(current_segment))
            current_segment = [coords[i]]  # Start new segment from this point
    
    # Add the last point and create final segment
    current_segment.append(coords[-1])
    if len(current_segment) >= 2:
        segments.append(LineString(current_segment))
    
    return segments

def add_fillet(line1, line2, radius, num_points=20):
    """
    Add a circular fillet between two LineStrings that meet at a point.
    
    Parameters:
    -----------
    line1 : LineString
        First line, whose end point connects to line2
    line2 : LineString
        Second line, whose start point connects to line1
    radius : float
        Radius of the fillet arc
    num_points : int
        Number of points to use for the arc approximation
    
    Returns:
    --------
    tuple : (trimmed_line1, arc, trimmed_line2)
        Three LineStrings forming the filleted connection
    """
     # Get the connection point and adjacent points
    p1 = np.array(line1.coords[-2])  # Second-to-last point of line1
    vertex = np.array(line1.coords[-1])  # Connection point
    p2 = np.array(line2.coords[1])  # Second point of line2
    
    # Verify that the lines actually connect
    if not np.allclose(vertex, line2.coords[0]):
        raise ValueError("Lines must connect at line1's end and line2's start")
    
    # Calculate direction vectors (normalized)
    v1 = p1 - vertex
    v1 = v1 / np.linalg.norm(v1)
    
    v2 = p2 - vertex
    v2 = v2 / np.linalg.norm(v2)
    
    # Calculate the angle between the vectors
    cos_angle = np.dot(v1, v2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Handle numerical errors
    angle = np.arccos(cos_angle)
    
    # Check if lines are collinear (no fillet needed)
    if np.isclose(angle, 0) or np.isclose(angle, np.pi):
        raise ValueError("Lines are collinear; no fillet can be added")
    
    # Calculate the distance from vertex to fillet tangent points
    half_angle = angle / 2
    distance = radius / np.tan(half_angle)
    
    # Calculate tangent points
    tangent1 = vertex + v1 * distance
    tangent2 = vertex + v2 * distance
    
    # Calculate the center of the fillet arc
    # The center is perpendicular to the bisector at distance 'radius'
    bisector = (v1 + v2) / np.linalg.norm(v1 + v2)
    center_distance = radius / np.sin(half_angle)
    center = vertex + bisector * center_distance
    
    # Calculate start and end angles for the arc
    start_vec = tangent1 - center
    end_vec = tangent2 - center
    
    start_angle = np.arctan2(start_vec[1], start_vec[0])
    end_angle = np.arctan2(end_vec[1], end_vec[0])
    
    # Always use the shorter arc path
    # Calculate the angular difference
    angle_diff = end_angle - start_angle
    
    # Normalize to [-pi, pi]
    if angle_diff > np.pi:
        angle_diff -= 2 * np.pi
    elif angle_diff < -np.pi:
        angle_diff += 2 * np.pi
    
    # Generate arc points along the shorter path
    angles = np.linspace(start_angle, start_angle + angle_diff, num_points)
    arc_points = [(center[0] + radius * np.cos(a), 
                   center[1] + radius * np.sin(a)) for a in angles]
    
    # Create trimmed lines
    # Trim line1: keep all points up to but not including the last, then add tangent1
    trimmed_line1_coords = list(line1.coords[:-1]) + [tuple(tangent1)]
    trimmed_line1 = LineString(trimmed_line1_coords)
    
    # Trim line2: add tangent2, then all points except the first
    trimmed_line2_coords = [tuple(tangent2)] + list(line2.coords[1:])
    trimmed_line2 = LineString(trimmed_line2_coords)
    
    # Create the arc
    arc = LineString(arc_points)
    
    return trimmed_line1, arc, trimmed_line2