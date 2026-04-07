import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from fieldpath.utils.multi_agent import split_field
import fieldpath.plot as fplt
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import shapely.plotting as spl
fplt.set_plt_darkmode(plt)

NUM_AGENTS = 3

test_fields = [
    # Rectangular test field
    Polygon([(3,0), (10,0), (10,5), (3,5)]),
    # U-shaped test field
    Polygon([(0,0), (12,0), (12,2), (4,3), (4,8), (0,8)]),
    # Complex field shaped polygon with at least 8 corners and no 90 deg corners and with multiple holes
    Polygon([(1,1), (9,2), (8,5), (6,6), (7,9), (3,8), (2,5), (0,4)],
            holes=[[(2,3), (4,2.5), (4,4), (3,4)],
                   [(6,3), (7,3), (7,4), (6,4)]]),

    # Field with hole
    Polygon([(0,0), (9,0), (9,10), (0,10)], holes=[[(3,3), (7,3), (7,4), (3,3)]]),
    # Diagonal field
    Polygon([(2,0), (10,4), (8,10), (0,6)]),
    # star-shaped field
    Polygon([(5,0), (6,3), (9,3), (7,5), (8,8), (5,6), (2,8), (3,5), (1,3), (4,3)]),
]

fig, axs = plt.subplots(2, 3) #plt.subplots(2, 3)
for i, ax in enumerate(axs.flatten()):
    plt.sca(ax)
    ax.set_aspect('equal')
    if test_fields and i < len(test_fields):
        segments = split_field(test_fields[i], NUM_AGENTS)
        for j, segment in enumerate(segments):
            spl.plot_polygon(segment, ax=ax, facecolor=f"C{j}", edgecolor=f"C{j}", color=f"C{j}", add_points=False, alpha=0.5)
        ax.set_title(f"Test Field {i+1} - Split into {len(segments)} segments")
        print("Test Field {}: Split into {} segments".format(i+1, len(segments)))
        print(segments)
    plt.tight_layout()
plt.tight_layout()
plt.show()

