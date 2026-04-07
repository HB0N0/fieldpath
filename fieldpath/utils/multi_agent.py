from shapely.geometry import Polygon, LineString
from shapely.ops import split
from shapely.affinity import rotate
from .geom import get_field_dominant_angle

def split_field(field_polygon, num_segments, cut_angle=None, cut_offset=90.0):
    """
    Split a field polygon into multiple segments for multi-agent coverage.

    :param field_polygon: Shapely Polygon representing the field
    :param num_segments: Number of segments to split into
    :param cut_angle: Optional angle for the cut line orientation, If None, it will be determined based on the field's dominant angle
    :param cut_offset: Offset angle for the cut line orientation (default is 90.0, so cuts are along the longest dimension. This reduces headland turns.)
    :return: List of shapely polgons
    """
    if field_polygon.is_empty or field_polygon.area < 1e-10:
        return []
    
    if cut_angle is None:
        cut_angle = get_field_dominant_angle(field_polygon)

    # Apply cut offset angle
    cut_angle = (cut_angle + cut_offset) % 360  
    
    # Get the centroid for rotation
    centroid = field_polygon.centroid
    
    # Rotate polygon to align with axes (perpendicular to cut angle)
    # The cut lines will be perpendicular to the cut direction
    rotated_polygon = rotate(field_polygon, -cut_angle, origin=centroid)
    
    minx, miny, maxx, maxy = rotated_polygon.bounds
    total_area = rotated_polygon.area
    target_area = total_area / num_segments
    
    # Create cut lines at positions that achieve nearly equal areas
    cut_lines = []
    accumulated_geoms = [rotated_polygon]
    
    for segment_idx in range(1, num_segments):
        # Find the x-position where the area to the left equals target_area * segment_idx
        target_accumulated_area = target_area * segment_idx
        
        # Binary search to find the x-position
        x_left = minx
        x_right = maxx
        tolerance = 1e-6
        
        while x_right - x_left > tolerance:
            x_mid = (x_left + x_right) / 2
            cut_line_test = LineString([(x_mid, miny - 10), (x_mid, maxy + 10)])
            
            try:
                # Calculate area to the left of the cut
                area_left = 0
                for geom in accumulated_geoms:
                    result = split(geom, cut_line_test)
                    if hasattr(result, 'geoms'):
                        for split_geom in result.geoms:
                            if isinstance(split_geom, Polygon):
                                # Check if this piece is to the left of the cut
                                if split_geom.centroid.x < x_mid:
                                    area_left += split_geom.area
                    else:
                        area_left += geom.area
                
                if area_left < target_accumulated_area:
                    x_left = x_mid
                else:
                    x_right = x_mid
            except:
                x_left = x_mid
        
        x_cut = (x_left + x_right) / 2
        cut_lines.append(LineString([(x_cut, miny - 10), (x_cut, maxy + 10)]))
    
    # Split sequentially with all cut lines
    segments = []
    current_geoms = [rotated_polygon]
    
    for cut_line in cut_lines:
        next_geoms = []
        for geom in current_geoms:
            try:
                result = split(geom, cut_line)
                if hasattr(result, 'geoms'):
                    for split_geom in result.geoms:
                        if isinstance(split_geom, Polygon) and split_geom.area > 1e-10:
                            next_geoms.append(split_geom)
                else:
                    next_geoms.append(geom)
            except:
                next_geoms.append(geom)
        current_geoms = next_geoms
    
    # Rotate all segments back to original orientation
    for geom in current_geoms:
        if isinstance(geom, Polygon) and geom.area > 1e-10:
            rotated_back = rotate(geom, cut_angle, origin=centroid)
            segments.append(rotated_back)
    
    return segments