import typing
from itertools import product

import numpy as np
from dataclasses import dataclass
from matplotlib import patches
from matplotlib.axes import Axes


class Point2d(typing.NamedTuple):
    x: float
    y: float

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y])


class Zone2d(typing.NamedTuple):
    x_min: float
    x_max: float
    y_min: float
    y_max: float


class RectangleObstacle(typing.NamedTuple):
    pos: Point2d
    height: float
    width: float

    def collides(self, point: Point2d):
        return (
                       self.pos.x - self.width / 2.0 <= point.x <= self.pos.x + self.width / 2.0
               ) and (
                       self.pos.y - self.height / 2.0 <= point.y <= self.pos.y + self.height / 2.0
               )

    def plot(self, axes: Axes):
        axes.add_patch(
            patches.Rectangle(
                xy=(self.pos.x - self.width / 2.0, self.pos.y - self.height / 2.0),
                height=self.height,
                width=self.width,
            )
        )


class Path(typing.NamedTuple):
    points: typing.List[Point2d]


@dataclass
class Trajectory:
    path: typing.List[Point2d]
    steering_input: typing.List[float]
    time: float


def collides(trajectory: Trajectory, obstacles: typing.List[RectangleObstacle]) -> bool:
    return any(
        obstacle.collides(path_point)
        for obstacle, path_point in product(obstacles, trajectory.path)
    )