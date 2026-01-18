import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from fieldpath.utils.dubins_path import DubinsPath
from shapely import LineString
import matplotlib.pyplot as plt
import numpy as np

# Define rectangle vertices
rect_size = 2

# Clockwise rectangle (right turns): bottom-left -> bottom-right -> top-right -> top-left -> bottom-left
clockwise_segments = [
    LineString([(0, 0), (rect_size, 0)]),          # bottom edge (left to right)
    LineString([(rect_size, 0), (rect_size, rect_size)]),  # right edge (bottom to top)
    LineString([(rect_size, rect_size), (0, rect_size)]),  # top edge (right to left)
    LineString([(0, rect_size), (0, 0)]),          # left edge (top to bottom)
]

# Counterclockwise rectangle (left turns): bottom-left -> top-left -> top-right -> bottom-right -> bottom-left
counterclockwise_segments = [
    LineString([(0, 0), (0, rect_size)]),          # left edge (bottom to top)
    LineString([(0, rect_size), (rect_size, rect_size)]),  # top edge (left to right)
    LineString([(rect_size, rect_size), (rect_size, 0)]),  # right edge (top to bottom)
    LineString([(rect_size, 0), (0, 0)]),          # bottom edge (right to left)
]

turn_radius = 0.5

# Plot both rectangles
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

rectangles = [
    ("Clockwise Rectangle", clockwise_segments, axes[0]),
    ("Counterclockwise Rectangle", counterclockwise_segments, axes[1]),
]

colors = ['blue', 'green', 'orange', 'purple']
dubins_colors = ['red', 'darkred', 'crimson', 'firebrick']

for title, segments, ax in rectangles:
    print(f"\n{title}:")
    print("-" * 40)
    
    # Plot all line segments
    for i, segment in enumerate(segments):
        x, y = segment.xy
        ax.plot(x, y, color=colors[i], linewidth=2, label=f'Edge {i+1}')
    
    # Generate Dubins path at each corner (between consecutive segments)
    for i in range(len(segments)):
        line1 = segments[i]
        line2 = segments[(i + 1) % len(segments)]  # wrap around to first segment
        
        dubins = DubinsPath(turn_radius=turn_radius)
        dubins.set_start(line1)
        dubins.set_end(line2)
        
        print(f"Corner {i+1}: start={dubins.start}, end={dubins.end}")
        
        path = dubins.generate_path()
        if path.length > 0:
            x, y = path.xy
            ax.plot(x, y, color=dubins_colors[i], linewidth=2, linestyle='--', 
                   label=f'Dubins {i+1} (L={path.length:.2f})')
            print(f"  Path length: {path.length:.3f}")
        else:
            print(f"  No path generated")
    
    ax.set_aspect('equal')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

plt.tight_layout()
plt.show()