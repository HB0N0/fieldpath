import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))
import matplotlib.pyplot as plt
from shapely import Polygon
from shapely.plotting import plot_polygon
from fieldpath.utils.convex_decomp import convex_decomp_sweep_line

def plot_decomposition(original: Polygon, sub_polygons: list[Polygon], title: str):
    """Visualize the decomposition result."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    
    # Original
    ax1.set_title("Original")
    plot_polygon(original, ax=ax1, add_points=False, facecolor='lightblue', edgecolor='blue')
    ax1.set_aspect('equal')
    
    # Decomposed
    ax2.set_title(f"Decomposed ({len(sub_polygons)} parts)")
    colors = plt.cm.Set3(range(len(sub_polygons)))
    for i, sub in enumerate(sub_polygons):
        plot_polygon(sub, ax=ax2, add_points=False, facecolor=colors[i], edgecolor='black', alpha=0.7)
    ax2.set_aspect('equal')
    
    fig.suptitle(title)
    plt.tight_layout()

convex_decomposition = convex_decomp_sweep_line

# Test 1: L-shape
l_shape = Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])
result1 = convex_decomposition(l_shape)
print(f"L-shape: {len(result1)} sub-polygons")
plot_decomposition(l_shape, result1, "L-shaped Polygon")

# Test 2: Plus shape
plus = Polygon([(1, 0), (2, 0), (2, 1), (3, 1), (3, 2), (2, 2),
                (2, 3), (1, 3), (1, 2), (0, 2), (0, 1), (1, 1)])
result2 = convex_decomposition(plus)
print(f"Plus-shape: {len(result2)} sub-polygons")
plot_decomposition(plus, result2, "Plus-shaped Polygon")

# Test 3: Staircase
stair = Polygon([(0, 0), (1, 0), (1, 1), (2, 1), (2, 2), (3, 2), (3, 3), (0, 3)])
result3 = convex_decomposition(stair)
print(f"Staircase: {len(result3)} sub-polygons")
plot_decomposition(stair, result3, "Staircase Polygon")

plt.show()
