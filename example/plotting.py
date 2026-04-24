import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import matplotlib.pyplot as plt
from shapely import Polygon

import fieldpath.plot as fpl
from fieldpath.planner import FieldPathPlanner

# Simple field geometry
field = Polygon([(0,0), (10,0), (10,5), (5,5), (5,10), (0,10)])

# Initialize FieldPathPlanner
fp = FieldPathPlanner(working_width=0.5, turn_radius=0.3)

# Generate working paths
working_path, sub_polygons = fp.generate_contoured_a_b_paths(field, num_contour_lines=2)

# Next generate work sequence
full_path, line_types = fp.generate_work_sequence(working_path, 
                                                  start_pose=(0,0,0), 
                                                  end_pose=(0,0,0), 
                                                  field=field)

# Set plot window to dark mode because it looks much cooler that way
fpl.set_plt_darkmode(plt)

# Plot field outline and sub-polygons (result of polygon decomposition)
fpl.plot_field(field, sub_polygons, ax=plt.gca())

# Plot the generated path, line_types specifies which segments are 'operation' vs 'traversal' for coloring
fpl.plot_working_path(full_path, ax=plt.gca(), types=line_types)

# Optional: show an animated point driving along the path
anim = fpl.animate_working_path(full_path, ax=plt.gca(), interval=20)

plt.title("Basic Field Path Planning Example")
plt.show()