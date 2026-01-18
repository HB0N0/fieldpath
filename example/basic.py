from shapely import Polygon
from fieldpath.planner import FieldPathPlanner

# Simple field geometry
field = Polygon([(0,0), (10,0), (10,5), (5,5), (5,10), (0,10)])

# Initialize FieldPathPlanner
fp = FieldPathPlanner(working_width=0.5, turn_radius=1.0)
# Generate operation paths
working_path = fp.generate_contoured_a_b_paths(field, 
                                               num_contour_lines=2)
# operation paths is a MultiLineString with all paths that need operation

# Next generate work sequence
full_path, line_types = fp.generate_work_sequence(working_path, 
                                                  start_pose=(0,0,0), 
                                                  end_pose=(0,0,0), 
                                                  field=field)

# full_path is a LineString with the complete path including transitions
# line_types is a list of strings indicating the type of each segment in full_path ('traversal', 'operation')
