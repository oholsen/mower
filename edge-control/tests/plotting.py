from math import cos, sin
from typing import List, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.path import Path
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point, Polygon, box

from edge_control.models.state import State

"""
Map presentation in Matplotlib, generally converting from Shapely shapes to Matplotlib paths and patches. 
"""


def polygon_path(points):
    # codes = [Path.MOVETO] + [Path.LINETO] * (len(points) - 1) + [Path.CLOSEPOLY]
    # vertices = points + [(0, 0)]
    codes = [Path.MOVETO] + [Path.LINETO] * (len(points) - 1)
    vertices = points
    vertices = np.array(vertices, float)
    return Path(vertices, codes, closed=True)


def shape_to_patch(shape, **kwargs):

    if isinstance(shape, Polygon):
        return patches.Polygon(list(shape.exterior.coords)[1:], **kwargs)

    if isinstance(shape, MultiPolygon):
        paths = [polygon_path(list(o.exterior.coords)[1:]) for o in shape.geoms]
        path = Path.make_compound_path(*paths)
        return patches.PathPatch(path, **kwargs)

    if isinstance(shape, LineString):
        path_data = [(Path.MOVETO, shape.coords[0])]
        for coord in shape.coords[1:]:
            path_data.append((Path.LINETO, coord))
        codes, verts = zip(*path_data)
        path = Path(verts, codes, closed=False)
        return patches.PathPatch(path, **kwargs)


def lines_to_patch(lines, **kwargs):
    path_data = []
    for line in lines:
        path_data.append((Path.MOVETO, line.coords[0]))
        path_data.append((Path.LINETO, line.coords[1]))
    codes, verts = zip(*path_data)
    path = Path(verts, codes)
    return patches.PathPatch(path, **kwargs)


class States:
    def __init__(self, marker: str = ".b", color="lightgrey"):
        self.marker = marker
        self.color = color
        self.x = []  # type: List[float]
        self.y = []  # type: List[float]
        self.theta = []  # type: List[float]
        self.state = None  # type: Optional[State]

    def update(self, state: State):
        self.state = state
        self.x.append(state.x)
        self.y.append(state.y)
        self.theta.append(state.theta)

    def plot(self, ax: plt.Axes):

        if self.state:
            ax.add_patch(
                patches.Circle(
                    (self.state.x, self.state.y),
                    0.3,
                    zorder=2,
                    facecolor=self.color,
                    edgecolor="black",
                )
            )
            a = 1  # * speed
            ax.arrow(
                self.state.x,
                self.state.y,
                a * cos(self.state.theta),
                a * sin(self.state.theta),
                zorder=2,
            )  # top

        ax.plot(self.x, self.y, self.marker, label="trajectory", zorder=1)  # bottom


class Plot:
    def __init__(self, states: List[States], ax: plt.Axes = None, frames_per_plot=1):
        self.states = states
        if ax is None:
            fig, ax = plt.subplots()
            fig.set_size_inches(10, 6)
        self.ax = ax
        self.patches = []  # type: List[patches.Patch]
        self.frame = 0
        self.frames_per_plot = frames_per_plot

    def show(self):
        self.render()
        plt.show()

    def pause(self):
        plt.pause(1e-9)

    def add_states(self, states: States):
        self.states.append(states)

    def add_shape(self, shape, **kwargs):
        """Add shapely Shape outline"""
        patch = shape_to_patch(shape, **kwargs)
        self.patches.append(patch)

    def render(self):
        self.frame += 1
        if self.frame < self.frames_per_plot:
            return
        self.frame = 0
        self._render()

    def _render(self):
        self.ax.cla()
        self.ax.set_aspect("equal", "box")
        self.ax.grid(True)
        for p in self.patches:
            self.ax.add_patch(p)

        for s in self.states:
            s.plot(self.ax)

        # GUI event loop
        plt.pause(0.001)
