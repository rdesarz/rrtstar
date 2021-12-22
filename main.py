import typing
import random

import numpy as np
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.axes import Axes


class Parameters(typing.NamedTuple):
    max_nb_iterations: int
    expand_dist: float
    goal_sample_rate: int


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
        return (self.pos.x - self.width / 2.0 <= point.x <= self.pos.x + self.width / 2.0) and (
                self.pos.y - self.height / 2.0 <= point.y <= self.pos.y + self.height / 2.0)

    def plot(self, axes: Axes):
        axes.add_patch(
            patches.Rectangle(xy=(self.pos.x - self.width / 2.0, self.pos.y - self.height / 2.0), height=self.height,
                              width=self.width))


class Environment(typing.NamedTuple):
    planification_zone: Zone2d
    obstacles: typing.List[RectangleObstacle]


class Path(typing.NamedTuple):
    points: typing.List[Point2d]


@dataclass
class Vertex:
    position: Point2d
    parent: typing.Optional['Vertex']


@dataclass
class Tree:
    vertices: typing.List[Vertex]


# Nearest vertex computation policies
def compute_nearest_euclidian_distance(new_sample: Point2d, graph: Tree) -> Vertex:
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


def generate_new_sample_biased_towards_goal(planification_zone: Zone2d, goal: Point2d,
                                            goal_sample_rate: int) -> Point2d:
    # There is a probability to generate a sample that is the goal. Therefore, the tree is biased to grow towards the goal.
    if random.randint(0, 100) > goal_sample_rate:
        return Point2d(
            random.uniform(planification_zone.x_min, planification_zone.x_max),
            random.uniform(planification_zone.y_min, planification_zone.y_max))
    else:
        return Point2d(goal.x, goal.y)


# Steering policies
def line_steering_policy(nearest: Point2d, random: Point2d, dist: float) -> Point2d:
    unit_vector = (random.to_array() - nearest.to_array())
    unit_vector = unit_vector / np.linalg.norm(unit_vector)
    new_position = nearest.to_array() + unit_vector * dist
    return Point2d(new_position[0], new_position[1])


def in_obstacles_zones(new_sample: Point2d, obstacles: typing.List[RectangleObstacle]) -> bool:
    return any(obstacle.collides(new_sample) for obstacle in obstacles)


def update(env: Environment, params: Parameters, goal: Point2d,
           steering_policy: typing.Callable[[Point2d, Point2d], Point2d],
           nearest_policy: typing.Callable[[Point2d, Tree], Vertex],
           sample_generation_policy: typing.Callable[[], Point2d], tree: Tree) -> bool:
    random_sample: Point2d = sample_generation_policy()

    nearest: Vertex = nearest_policy(random_sample, tree)
    new_sample = steering_policy(nearest.position, random_sample)

    if in_obstacles_zones(new_sample, env.obstacles):
        return False

    tree.vertices.append(Vertex(position=new_sample, parent=nearest))

    if np.linalg.norm(new_sample.to_array() - goal) < params.expand_dist:
        return True

    return False


def update_plot(env: Environment, start: Point2d, goal: Point2d, tree: Tree, axes: Axes):
    # Plot start and goal
    axes.plot(start.x, start.y, '*')
    axes.plot(goal.x, goal.y, '*')

    # Plot obstacles
    for obstacle in env.obstacles:
        obstacle.plot(axes)

    # Plot graph using BFS
    for vertex in tree.vertices:
        if vertex.parent:
            x_values = [vertex.position.x, vertex.parent.position.x]
            y_values = [vertex.position.y, vertex.parent.position.y]
            plt.plot(x_values, y_values, 'b')

    current = tree.vertices[-1]
    while current.parent:
        x_values = [current.position.x, current.parent.position.x]
        y_values = [current.position.y, current.parent.position.y]
        plt.plot(x_values, y_values, 'r')
        current = current.parent


def main():
    # Initialize environment
    planification_zone = Zone2d(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0)
    obstacles = [RectangleObstacle(pos=Point2d(x=5.0, y=4.0), width=1.0, height=1.0),
                 RectangleObstacle(pos=Point2d(x=6.5, y=5.5), width=1.0, height=1.0)]
    environment = Environment(planification_zone=planification_zone, obstacles=obstacles)

    # Set start and goal position
    start = Point2d(x=2.0, y=2.0)
    goal = Point2d(x=8.0, y=6.0)

    # Set parameters
    parameters = Parameters(max_nb_iterations=500, expand_dist=0.2, goal_sample_rate=20)

    tree = Tree(vertices=[Vertex(position=start, parent=None)])

    plt.figure()
    axes = plt.subplot()

    def steering_policy(nearest, random):
        return line_steering_policy(nearest, random, parameters.expand_dist)

    def nearest_policy(new, tree):
        return compute_nearest_euclidian_distance(new, tree)

    def sample_generation_policy():
        return generate_new_sample_biased_towards_goal(environment.planification_zone, goal,
                                                       parameters.goal_sample_rate)

    for iteration in range(0, parameters.max_nb_iterations):
        if update(environment, parameters, goal, steering_policy, nearest_policy, sample_generation_policy, tree):
            break

    update_plot(environment, start, goal, tree, axes)

    plt.show()


if __name__ == '__main__':
    main()
