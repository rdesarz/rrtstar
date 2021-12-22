import typing

import numpy as np
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.axes import Axes


class Parameters(typing.NamedTuple):
    max_nb_iterations: int
    expand_dist: float


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


@dataclass
class Vertex:
    position: Point2d
    connected: typing.List


@dataclass
class Graph:
    vertices: typing.List[Vertex]


# Nearest vertex computation policies
def compute_nearest_euclidian_distance(new_sample: Point2d, graph: Graph) -> Vertex:
    norms = [np.linalg.norm(vertex.position.to_array() - new_sample.to_array()) for vertex in graph.vertices]

    return graph.vertices[norms.index(min(norms))]


# Sample generation policies
def generate_new_sample_uniform(planification_zone: Zone2d) -> Point2d:
    x = np.random.uniform(planification_zone.x_min, planification_zone.x_max, 1)
    y = np.random.uniform(planification_zone.y_min, planification_zone.y_max, 1)
    return Point2d(x[0], y[0])


def generate_new_sample_biased(goal: Point2d) -> Point2d:
    x, y = np.random.multivariate_normal(goal.to_array(), [[10, 0], [0, 10]]).T
    return Point2d(x, y)


# Steering policies
def line_steering_policy(nearest: Point2d, random: Point2d, dist: float) -> Point2d:
    unit_vector = (random.to_array() - nearest.to_array())
    unit_vector = unit_vector / np.linalg.norm(unit_vector)
    new_position = nearest.to_array() + unit_vector * dist
    return Point2d(new_position[0], new_position[1])


def in_obstacles_zones(new_sample: Point2d, obstacles_zones: typing.List[Zone2d]) -> bool:
    return False


def update(env: Environment, goal_position: Point2d, steering_policy: typing.Callable[[Point2d, Point2d], Point2d],
           nearest_policy: typing.Callable[[Point2d, Graph], Vertex],
           sample_generation_policy: typing.Callable[[], Point2d], graph: Graph):
    random_sample: Point2d = sample_generation_policy()

    if in_obstacles_zones(random_sample, env.obstacles_zones):
        return

    nearest: Vertex = nearest_policy(random_sample, graph)
    new_sample = steering_policy(nearest.position, random_sample)

    graph.vertices.append(Vertex(position=new_sample, connected=[]))
    nearest.connected.append(graph.vertices[-1])


def update_plot(env: Environment, start_position: Point2d, goal_position: Point2d, graph: Graph, axes: Axes):
    axes.plot(start_position.x, start_position.y, '*')
    axes.plot(goal_position.x, goal_position.y, '*')

    # Plot graph using BFS
    visited = [graph.vertices[0]]
    stack = [graph.vertices[0]]
    while not len(stack) == 0:
        vertex = stack.pop()

        for neigh in vertex.connected:
            if neigh not in visited:
                x_values = [vertex.position.x, neigh.position.x]
                y_values = [vertex.position.y, neigh.position.y]
                plt.plot(x_values, y_values, 'b')

                visited.append(neigh)
                stack.append(neigh)


def main():
    # Initialize environment
    planification_zone = Zone2d(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0)
    obstacles_zones = []
    environment = Environment(planification_zone=planification_zone, obstacles_zones=obstacles_zones)

    # Set start and goal position
    start = Point2d(x=2.0, y=2.0)
    goal = Point2d(x=8.0, y=6.0)

    # Set parameters
    parameters = Parameters(max_nb_iterations=1000, expand_dist=0.3)

    graph = Graph(vertices=[Vertex(position=start, connected=[])])

    plt.figure()
    axes = plt.subplot()

    def steering_policy(nearest, random):
        return line_steering_policy(nearest, random, parameters.expand_dist)

    def nearest_policy(new, graph):
        return compute_nearest_euclidian_distance(new, graph)

    def sample_generation_policy():
        return generate_new_sample_biased(goal)

    for iteration in range(0, parameters.max_nb_iterations):
        update(environment, goal, steering_policy, nearest_policy, sample_generation_policy, graph)

    update_plot(environment, start, goal, graph, axes)

    plt.show()


if __name__ == '__main__':
    main()
