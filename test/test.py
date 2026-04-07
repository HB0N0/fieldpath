import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from fieldpath.planner import *
import fieldpath.plot as fplt
from fieldpath.stats import print_metrics
fplt.set_plt_darkmode(plt)
plt.rcParams['animation.ffmpeg_path'] = 'C:\\Users\\hanne\\AppData\\Local\\Microsoft\\WinGet\\Packages\\Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\\ffmpeg-8.0.1-full_build\\bin\\ffmpeg.exe'

WORKING_WIDTH = 0.3
TURN_RADIUS = 0.1
NUM_CONTOUR_LINES = 2
CONTOUR_ORDER = 'last'  # 'first'  # 'last'  # 'auto'  #
ENABLE_POLYGON_DECOMPOSITION = True

WINDOW_PER_PLOT = False

start_pose = (0, 0, 0)
end_pose = (0, 0, 0)

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

anims = []  # keep animation objects alive
def generate_plot(fig, ax, field, index):
    plt.sca(ax)
    plt.title(f"Test Field {index+1}")
    print(f"Generating Test Field {index+1} =====")

    fp = FieldPathPlanner(working_width=WORKING_WIDTH, turn_radius=TURN_RADIUS)
    lines, polygon_segments = fp.generate_contoured_a_b_paths(field, num_contour_lines=NUM_CONTOUR_LINES, use_polygon_decomposition=ENABLE_POLYGON_DECOMPOSITION)

    path, line_types = fp.generate_work_sequence(   lines, 
                                                    start_pose=start_pose,
                                                    end_pose=end_pose,
                                                    field=field, 
                                                    contour_order=CONTOUR_ORDER)

    fplt.plot_field(field, polygon_segments, ax)
    fplt.plot_working_path(path, ax, types=line_types)

    print_metrics(path, field, line_types=line_types, working_width=fp.working_width)

    anim = fplt.animate_working_path(path, ax, interval=20)

    # export animation to gif
    # fig.set_size_inches(10.24, 7.68, True)  # 1024x768 pixels
    plt.tight_layout()
    # ffmpeg_writer = animation.FFMpegWriter(fps=20)
    # anim.save(f"animation_{index+1}.mp4", writer=ffmpeg_writer, dpi=100)
    anims.append(anim)

if WINDOW_PER_PLOT:
    for i, field in enumerate(test_fields):
        if i not in [0,1]: continue
        fig, ax = plt.subplots()
        generate_plot(fig, ax, field, i)
    plt.tight_layout()
    plt.show()
else:
    fig, axs = plt.subplots(2, 3) #plt.subplots(2, 3)
    for i, ax in enumerate(axs.flatten()):
        if test_fields and i < len(test_fields):
            generate_plot(fig, ax, test_fields[i], i)

    plt.tight_layout()
    plt.show()