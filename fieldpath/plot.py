from matplotlib.animation import FuncAnimation
from shapely import Polygon
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
import shapely.plotting as spl

import fieldpath.planner as planner

def plot_field(field: Polygon, sub_parcels: list = [], ax = None):
    if ax is None:
        fig, ax = plt.subplots()
    spl.plot_polygon(field, ax=ax, facecolor="#5a8a5a78", edgecolor='#90ee90', color='#5a8a5a', add_points=False, alpha=0.5)
    for sub in sub_parcels:
        spl.plot_polygon(sub, ax=ax, facecolor="#5a8a5a00", edgecolor='#7bc77b', color='#5a8a5a', add_points=False, alpha=0.2)

def plot_working_path(path: planner.MultiLineString, ax = None, types = []):
    if ax is None:
        fig, ax = plt.subplots()
    waypoint_number = 0
    last_waypoint = None

    # Color the lines by cumulative distance traveled
    total_length = sum(line.length for line in path.geoms)
    cumulative_distance = 0
    colors = cm.get_cmap('viridis')
    
    for i, line in enumerate(path.geoms):
        # Use cumulative distance as a fraction of total path length
        color_value = cumulative_distance / total_length if total_length > 0 else 0

        line_type = types[i] if i < len(types) else 'operation'
        linestyle = 'dotted' if line_type == 'traversal' else 'solid'
        line_color = "#7F86A6" if line_type == 'traversal' else colors(color_value)
        line_width = 1.0 if line_type == 'traversal' else 2.0

        spl.plot_line(line, color=line_color, linestyle=linestyle, linewidth=line_width, add_points=False)
        cumulative_distance += line.length
        
        # Number the points
        for point in line.coords:
            if last_waypoint is None or point != last_waypoint:
                # ax.text(point[0], point[1], str(waypoint_number), fontsize=8, color='#cccccc')
                waypoint_number += 1
                last_waypoint = point

def animate_working_path(path, ax=None, interval=50, marker_size=6):
    """Animate a point traveling along the MultiLineString `path`."""
    if ax is None:
        fig, ax = plt.subplots()

    # Flatten all segments from the multilines
    segments, seg_lengths = [], []
    for line in path.geoms:
        coords = list(line.coords)
        for a, b in zip(coords[:-1], coords[1:]):
            pa, pb = np.array(a, dtype=float), np.array(b, dtype=float)
            segments.append((pa, pb))
            seg_lengths.append(np.linalg.norm(pb - pa))

    if not segments:
        raise ValueError("Path is empty; nothing to animate.")
    cumulative = np.cumsum([0.0] + seg_lengths)
    total_len = cumulative[-1]

    # Moving point
    (dot,) = ax.plot([], [], "o", color='#ff6b6b', markersize=marker_size)

    def interp_at(distance):
        idx = np.searchsorted(cumulative, distance, side="right") - 1
        idx = min(idx, len(segments) - 1)
        start, end = segments[idx]
        seg_len = seg_lengths[idx]
        if seg_len == 0:
            return start
        t = (distance - cumulative[idx]) / seg_len
        return start + t * (end - start)

    def update(distance):
        x, y = interp_at(distance)
        dot.set_data([x], [y])
        return (dot,)
    
    frames = np.linspace(0, total_len, max(2, int(total_len * 10)))  # ~10 steps per unit length
    ani = FuncAnimation(ax.figure, update, frames=frames, interval=interval, blit=True, repeat=True)
    return ani

def set_plt_darkmode(plt):
    plt.style.use('dark_background')
    plt.rcParams['figure.facecolor'] = "#0d0e12"
    plt.rcParams['axes.facecolor'] = "#181a20"
    plt.rcParams['axes.labelcolor'] = "#a9b1d6"
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.color'] = "#414868"
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['xtick.labelcolor'] = "#787c99"
    plt.rcParams['xtick.color'] = "#414868"
    plt.rcParams['xtick.labelsize'] = "small"
    plt.rcParams['ytick.labelcolor'] = "#787c99"
    plt.rcParams['ytick.color'] = "#414868"
    plt.rcParams['ytick.labelsize'] = "small"
    plt.rcParams['text.color'] = "#c0caf5"
    plt.rcParams['axes.titlecolor'] = "#7aa2f7"
    plt.rcParams['legend.facecolor'] = "#1a1b26"
    plt.rcParams['legend.edgecolor'] = "#414868"
