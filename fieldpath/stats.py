from shapely import MultiLineString, Polygon, union_all

class CostFunctions:
    def waypoint_cost(path: MultiLineString) -> float:
        """Calculate cost based on number of waypoints"""
        count = [len(line.coords) for line in path.geoms]
        return sum(count)
    
    def path_length_cost(path: MultiLineString) -> float:
        """Calculate cost based on total path length"""
        return sum(line.length for line in path.geoms)
    
    def crossing_cost(path: MultiLineString) -> float:
        """Calculate cost based on number of crossings (intersections) in the path
        Only crossings with already driven lines are counted"""
        crossings = 0
        lines = list(path.geoms)
        for i in range(len(lines)):
            for j in range(0, i + 1):
                if lines[i].crosses(lines[j]):
                    crossings += 1
        return crossings
    
    def traversal_percentage_cost(path: MultiLineString, line_types: list) -> float:
        """Calculate cost based on percentage of traversal segments in the path"""
        traversal_length = 0.0
        total_length = 0.0
        for i, line in enumerate(path.geoms):
            total_length += line.length
            if line_types[i] == 'traversal':
                traversal_length += line.length

        return traversal_length / total_length if total_length > 0 else 0
    
    def area_coverage_cost(path: MultiLineString, field: Polygon, path_width: float, line_types: list) -> float:
        """Calculate cost based on area coverage efficiency"""
        operated_areas = []
        for i, line in enumerate(path.geoms):
            if line_types[i] == 'operation':
                line_area = line.buffer(path_width / 2, cap_style='flat')
                operated_areas.append(line_area)
        # Ideally, we would compute the union of all operated areas and compare with field area
        operated_area = union_all(operated_areas)  # Placeholder for union operation
        intersection_area = operated_area.intersection(field).area
        return intersection_area / field.area if field.area > 0 else 0
    
def print_metrics(path: MultiLineString, field: Polygon, line_types: list = [], working_width: float = 1.0):
    """Print various metrics of the given path."""
    num_waypoints = CostFunctions.waypoint_cost(path)
    total_length = CostFunctions.path_length_cost(path)
    num_crossings = CostFunctions.crossing_cost(path)
    coverage = CostFunctions.area_coverage_cost(path, field, path_width=working_width, line_types=line_types)
    traversal_pct = CostFunctions.traversal_percentage_cost(path, line_types)

    field_area = field.area
    print(f"\tNumber of waypoints: {num_waypoints}")
    print(f"\tTotal path length: {total_length:.2f} units")
    print(f"\tNumber of crossings: {num_crossings}")
    print(f"\tField area: {field_area:.2f} square units")
    print(f"\tArea coverage efficiency: {coverage*100:.2f}%")
    print(f"\tTraversal distance percentage: {traversal_pct*100:.2f}%")