import typing

import numpy as np
from dataclasses import dataclass


class Parameters(typing.NamedTuple):
    max_nb_iterations: int


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


class Environment(typing.NamedTuple):
    planification_zone: Zone2d
    obstacles_zones: typing.List[Zone2d]


class Path(typing.NamedTuple):
    points: typing.List[Point2d]


def plot_iteration(graph):
    pass


@dataclass
class Vertex:
    position: Point2d
    connected: typing.List


@dataclass
class Graph:
    vertices: typing.List[Vertex]


def generate_new_sample(planification_zone: Zone2d) -> Point2d:
    x = np.random.uniform(planification_zone.x_min, planification_zone.x_max, 1)
    y = np.random.uniform(planification_zone.y_min, planification_zone.y_max, 1)
    return Point2d(x[0], y[0])


def is_in_obstacles_zones(new_sample: Point2d, obstacles_zones: typing.List[Zone2d]) -> bool:
    return False


def compute_nearest(new_sample: Point2d, graph: Graph) -> Point2d:
    min = 20000000.0
    min_element = None
    for vertex in graph.vertices:
        dist = np.linalg.norm(vertex.position.to_array() - new_sample.to_array())
        if dist < min:
            min = dist
            min_element = vertex.position

    return min_element


def update_graph(env: Environment, start_position: Point2d, goal_position: Point2d, graph: Graph):
    new_sample: Point2d = generate_new_sample(env.planification_zone)
    if is_in_obstacles_zones(new_sample, env.obstacles_zones):
        return

    nearest = compute_nearest(new_sample, graph)


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
