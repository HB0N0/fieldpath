
from shapely import Polygon, MultiPolygon, LineString
from shapely.affinity import rotate
from shapely.ops import split


def convex_decomp_sweep_line(polygon, sweep_angle: float = 0.0, min_width: float = 0.0) -> list[Polygon]:
    """
    Trapezoidal decomposition (sweep line method).
    
    Sweeps a line across the polygon and splits at each vertex,
    creating convex trapezoid-shaped sub-polygons.
    
    :param polygon: Input polygon or MultiPolygon to decompose
    :param sweep_angle: Angle of sweep direction in degrees (0 = vertical sweep)
    :param min_width: Minimum width between cuts (default: working_width). 
                        Prevents excessive splitting on rounded edges.
    :return: List of convex sub-polygons
    """
    
    # Handle MultiPolygon by decomposing each polygon separately
    if isinstance(polygon, MultiPolygon):
        result = []
        for geom in polygon.geoms:
            result.extend(convex_decomp_sweep_line(geom, sweep_angle, min_width))
        return result
    
    # Handle non-Polygon types
    if not isinstance(polygon, Polygon):
        return []
    
    if polygon.is_empty or polygon.area < 1e-10:
        return []
    
    # Rotate polygon to align sweep direction with vertical
    rotated = rotate(polygon, -sweep_angle, origin='centroid')
    
    # Get all vertex x-coordinates (sweep positions)
    coords = list(rotated.exterior.coords)[:-1]
    x_coords = sorted(set(c[0] for c in coords))
    
    # Filter x-coordinates to maintain minimum spacing
    if len(x_coords) > 2 and min_width > 0:
        filtered_x = [x_coords[0]]  # Always keep first
        for x in x_coords[1:-1]:
            if x - filtered_x[-1] >= min_width:
                filtered_x.append(x)
        filtered_x.append(x_coords[-1])  # Always keep last
        x_coords = filtered_x
    
    # Create vertical cut lines at each unique x-coordinate (except first/last)
    minx, miny, maxx, maxy = rotated.bounds
    margin = (maxy - miny) * 0.1
    
    sub_polygons = [rotated]
    
    for x in x_coords[1:-1]:  # Skip boundary vertices
        new_subs = []
        cut_line = LineString([(x, miny - margin), (x, maxy + margin)])
        
        for sub in sub_polygons:
            if sub.is_empty or sub.area < 1e-10:
                continue
            try:
                result = split(sub, cut_line)
                if hasattr(result, 'geoms'):
                    new_subs.extend([g for g in result.geoms 
                                    if isinstance(g, Polygon) and g.area > 1e-10])
                else:
                    new_subs.append(sub)
            except:
                new_subs.append(sub)
        
        sub_polygons = new_subs
    
    # Rotate back to original orientation
    result = []
    for sub in sub_polygons:
        rotated_back = rotate(sub, sweep_angle, origin=polygon.centroid)
        if rotated_back.area > 1e-10:
            result.append(rotated_back)
    return result