import matplotlib.pyplot as plt
from matplotlib import style
import pickle
from pprint import pprint

style.use('fivethirtyeight')

def load_base_map(filename=None):
    """Load (native cadvas) file, return list of geom line coords (cm)

    coordinates returned are a tuple of 2 end points of line segment
    where each point is a tuple of X, Y coordinates
    ((X1, Y1), (X2, Y2))
    X and Y coordinate values are converted from mm to cm.
    """

    if not filename:
        filename = 'map-k.pkl'
    with open(filename, 'rb') as file:
        data = pickle.load(file)

    coordlist = [((v[0][0][0]/10, v[0][0][1]/10),
                  (v[0][1][0]/10, v[0][1][1]/10))
                 for entity in data
                 for k, v in entity.items()
                 if k == 'gl']

    return coordlist

def plot_scan(pointlist, target=None):
    """Simple no-frills plot of points in pointlist,
    but rotated 90 deg CCW so Y axis is straight ahead."""
    # build data lists to plot scan points
    xs = []
    ys = []
    for point in pointlist:
        x, y = point
        # rotate plot 90 deg CCW
        xs.append(-y)
        ys.append(x)
    plt.scatter(xs, ys, color='#003F72')
    if target:
        x, y = target
        # rotate point 90 deg CCW
        cx = [-y]
        cy = [x]
        plt.scatter(cx, cy, color='#FFA500')
    # show origin at (0, 0)
    plt.scatter([0], [0], color='#FF0000')
    plt.axis('equal')
    plt.show()  # shows interactive plot
    plt.clf()  # clears previous points & lines

def plot(scanpoints, map_lines=None, target=None, waypoints=None,
         carspot=None, map_folder="Maps", seq_nmbr=None,
         show=True, display_all_points=True):
    """Plot all points and line segments and save in map_folder.

    Optional args:
    display_all_points=False to plot only points in regions
    seq_nmbr appended to save_file_name
    show=True to display an interactive plot (which blocks program).
    """

    filename = f"{map_folder}/scanMap"
    if seq_nmbr:
        str_seq_nmbr = str(seq_nmbr)
        # prepend a '0' to single digit values (helps viewer sort) 
        if len(str_seq_nmbr) == 1:
            str_seq_nmbr = '0' + str_seq_nmbr
        imagefile = filename + str_seq_nmbr + ".png"
    else:
        imagefile = filename + ".png"

    #fig = plt.figure()
    #ax = fig.add_axes([0, 0, 1, 1])  # all must be between 0 and 1
    #ax.set_xlim(-400, 800)
    #ax.set_ylim(-400, 800)

    # build data lists to plot scan points
    xs = []
    ys = []
    for point in scanpoints:
        x, y = point
        xs.append(x)
        ys.append(y)
    plt.scatter(xs, ys, color='#003F72')

    # plot previous waypoints
    if waypoints:
        xs = []
        ys = []
        for waypoint in waypoints:
            x, y = waypoint
            xs.append(x)
            ys.append(y)
            plt.scatter(xs, ys, color='#00FF00')

    # plot location of car on map
    if carspot:
        x, y = carspot
        cx = [x]
        cy = [y]
        plt.scatter(cx, cy, color='#FF0000')

    # plot location of target on map
    if target:
        x, y = target
        cx = [x]
        cy = [y]
        plt.scatter(cx, cy, color='#FFA500')

    # plot map
    for segment in map_lines:
        x_vals = [segment[0][0], segment[1][0]]
        y_vals = [segment[0][1], segment[1][1]]
        plt.plot(x_vals, y_vals, linewidth=1, color="k")

    plt.axis('equal')
    plt.savefig(imagefile)

    if show:
        plt.show()  # shows interactive plot
    plt.clf()  # clears previous points & lines
