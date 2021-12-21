import typing

import numpy as np
from dataclasses import dataclass


class Parameters(typing.NamedTuple):
    max_nb_iterations: int


class Point2d(typing.NamedTuple):
    x: float
    y: float


class Zone2d(typing.NamedTuple):
    x_min: float
    x_max: float
    y_min: float
    y_max: float


class Environment(typing.NamedTuple):
    planification_zone: Zone2d
    obstacles_zones: typing.List[Zone2d]


class Path(typing.NamedTuple):
    points: typing.List[Point2d]


def plot_iteration(graph):
    pass


# @dataclass
# class Graph(typing.NamedTuple):


def generate_new_sample(planification_zone: Zone2d) -> Point2d:
    x = np.random.uniform(planification_zone.x_min, planification_zone.x_max, 1)
    y = np.random.uniform(planification_zone.y_min, planification_zone.y_max, 1)
    return Point2d(x[0], y[0])


def update_graph(planification_zone: Zone2d, start_position: Point2d, goal_position: Point2d, graph):
    new_sample = generate_new_sample(planification_zone)


def main():
    # Initialize environment
    planification_zone = Zone2d(x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0)
    obstacles_zones = []
    environment = Environment(planification_zone=planification_zone, obstacles_zones=obstacles_zones)

    # Set start and goal position
    start_position = Point2d(x=2.0, y=2.0)
    goal_position = Point2d(x=8.0, y=6.0)

    # Set parameters
    parameters = Parameters(max_nb_iterations=100)

    graph = Graph()

    for iteration in range(0, parameters.max_nb_iterations):
        update_graph(planification_zone, start_position, goal_position, graph)
        plot_iteration(graph)


if __name__ == '__main__':
    main()
