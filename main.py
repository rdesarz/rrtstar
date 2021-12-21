import typing

import numpy as np
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.axes import Axes


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


def compute_nearest(new_sample: Point2d, graph: Graph) -> Vertex:
    min = 20000000.0
    min_element = None
    for vertex in graph.vertices:
        dist = np.linalg.norm(vertex.position.to_array() - new_sample.to_array())
        if dist < min:
            min = dist
            min_element = vertex

    return min_element


def apply_motion_model(nearest: Point2d, random: Point2d, dist: float) -> Point2d:
    unit_vector = (random.to_array() - nearest.to_array())
    unit_vector = unit_vector / np.linalg.norm(unit_vector)
    new_position = nearest.to_array() + unit_vector * dist
    return Point2d(new_position[0], new_position[1])


def update_graph(env: Environment, start_position: Point2d, goal_position: Point2d, graph: Graph):
    random_sample: Point2d = generate_new_sample(env.planification_zone)
    if is_in_obstacles_zones(random_sample, env.obstacles_zones):
        return

    nearest: Vertex = compute_nearest(random_sample, graph)
    new_sample = apply_motion_model(nearest.position, random_sample, 0.3)

    graph.vertices.append(Vertex(position=new_sample, connected=[]))
    nearest.connected.append(graph.vertices[-1])


def update_plot(env: Environment, start_position: Point2d, goal_position: Point2d, graph: Graph, axes: Axes):
    axes.plot(start_position.x, start_position.y, '*')
    axes.plot(goal_position.x, goal_position.y, '*')

    visited = {graph.vertices[0]}
    stack = [graph.vertices[0]]

    while not len(stack) == 0:
        vertex = stack.pop()

        for neigh in vertex.connected:
            if neigh not in visited:
                x_values = [vertex.position.x, neigh.position.x]
                y_values = [vertex.position.y, neigh.position.y]
                plt.plot(x_values, y_values)

                visited.add(neigh)
                stack.append(neigh)


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

    graph = Graph(vertices=[Vertex(position=start_position, connected=[])])

    plt.figure()
    axes = plt.subplot()

    for iteration in range(0, parameters.max_nb_iterations):
        update_graph(environment, start_position, goal_position, graph)

    update_plot(environment, start_position, goal_position, graph, axes)

    plt.show()


if __name__ == '__main__':
    main()
