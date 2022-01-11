import math
import typing
import random
from itertools import product
import sys

import numpy as np
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.axes import Axes


class Parameters(typing.NamedTuple):
    max_nb_iterations: int
    expand_dist: float
    goal_sample_rate: int
    path_sampling_step: float
    time_to_steer: float
    velocity: float


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


class Environment(typing.NamedTuple):
    planification_zone: Zone2d
    obstacles: typing.List[RectangleObstacle]


class Path(typing.NamedTuple):
    points: typing.List[Point2d]


@dataclass
class Trajectory:
    path: typing.List[Point2d]
    steering_input: typing.List[float]
    time: float


@dataclass
class Vertex:
    position: Point2d
    trajectory: Trajectory
    parent: typing.Optional["Vertex"] = None
    cost: float = 0.0


@dataclass
class Tree:
    vertices: typing.List[Vertex]


# Nearest vertex computation policies
def compute_nearest_euclidian_distance(new_sample: Point2d, graph: Tree) -> Vertex:
    norms = [
        np.linalg.norm(vertex.position.to_array() - new_sample.to_array())
        for vertex in graph.vertices
    ]

    return graph.vertices[norms.index(min(norms))]


# Sample generation policies
def generate_new_sample_uniform(planification_zone: Zone2d) -> Point2d:
    x = np.random.uniform(planification_zone.x_min, planification_zone.x_max, 1)
    y = np.random.uniform(planification_zone.y_min, planification_zone.y_max, 1)
    return Point2d(x[0], y[0])


def generate_new_sample_biased(goal: Point2d) -> Point2d:
    x, y = np.random.multivariate_normal(goal.to_array(), [[10, 0], [0, 10]]).T
    return Point2d(x, y)


def generate_new_sample_biased_towards_goal(
    planification_zone: Zone2d, goal: Point2d, goal_sample_rate: int
) -> Point2d:
    # There is a probability to generate a sample that is the goal.
    # Therefore, the tree is biased to grow towards the goal.
    if random.randint(0, 100) > goal_sample_rate:
        return Point2d(
            random.uniform(planification_zone.x_min, planification_zone.x_max),
            random.uniform(planification_zone.y_min, planification_zone.y_max),
        )
    else:
        return Point2d(goal.x, goal.y)


# Steering policies
def constant_speed_line_steering_policy(
    start: Vertex, dest: Point2d, time: float, velocity: float
) -> typing.Tuple[float, Trajectory]:
    unit_vector = dest.to_array() - start.position.to_array()
    norm_vec = np.linalg.norm(unit_vector)
    unit_vector = unit_vector / norm_vec

    step = 0.05  # [s]
    timesteps = np.arange(start=0.0, stop=time + step, step=step)

    # Steer to the destination
    path = []
    timesteps = []
    for timestep in np.arange(start=0.0, stop=time + step, step=step):
        if velocity * timestep < norm_vec:
            path.append(
                Point2d(
                    start.position.to_array()[0] + unit_vector[0] * velocity * timestep,
                    start.position.to_array()[1] + unit_vector[1] * velocity * timestep,
                )
            )

            timesteps.append(timestep)
        else:
            path.append(dest)
            timesteps.append(timestep)

    # Generate trajectory. Time is equivalent to the steering input
    trajectory = Trajectory(path=path, steering_input=timesteps, time=timesteps[-1])

    cost = start.cost + np.linalg.norm(path[-1].to_array() - path[0].to_array())

    return cost, trajectory


def collides(trajectory: Trajectory, obstacles: typing.List[RectangleObstacle]) -> bool:
    return any(
        obstacle.collides(path_point)
        for obstacle, path_point in product(obstacles, trajectory.path)
    )


def find_near_vertices(
    tree: Tree, new_vertex: Vertex, expand_dist: float, near_dist: float
) -> typing.List[Vertex]:
    n_vertices = len(tree.vertices)
    dist_range = min(
        near_dist * math.sqrt(math.log(n_vertices) / n_vertices), expand_dist
    )

    return [
        vertex
        for vertex in tree.vertices
        if np.linalg.norm(vertex.position.to_array() - new_vertex.position.to_array())
        < dist_range
    ]


def find_optimal_parent(
    new_vertex: Vertex,
    near_vertices: typing.List[Vertex],
    steering_policy,
    obstacles: typing.List[RectangleObstacle],
) -> typing.Tuple[float, typing.Optional[Vertex]]:
    cost_min = sys.float_info.max
    vertex_min = None
    for near_vertex in near_vertices:
        cost_to_new, traj_to_new = steering_policy(near_vertex, new_vertex.position)
        if (
            not collides(trajectory=traj_to_new, obstacles=obstacles)
            and traj_to_new.path[-1] == near_vertex.position
        ):
            if near_vertex.cost + cost_to_new < cost_min:
                cost_min = near_vertex.cost + cost_to_new
                vertex_min = near_vertex

    return cost_min, vertex_min


def rewire(
    tree: Tree,
    new_vertex: Vertex,
    near_vertices: typing.List[Vertex],
    steering_policy,
    obstacles: typing.List[RectangleObstacle],
):
    for near_vertex in near_vertices:
        cost_to_near, traj_to_near = steering_policy(new_vertex, near_vertex.position)
        if (
            not collides(trajectory=traj_to_near, obstacles=obstacles)
            and traj_to_near.path[-1] == near_vertex.position
            and new_vertex.cost + cost_to_near < near_vertex.cost
        ):
            near_vertex.parent = new_vertex
            near_vertex.cost = new_vertex.cost + cost_to_near


def update(
    env: Environment,
    params: Parameters,
    goal: Point2d,
    steering_policy: typing.Callable[
        [Vertex, Point2d], typing.Tuple[float, Trajectory]
    ],
    nearest_policy: typing.Callable[[Point2d, Tree], Vertex],
    sample_generation_policy: typing.Callable[[], Point2d],
    tree: Tree,
) -> bool:
    # Generate new sample
    new_sample: Point2d = sample_generation_policy()

    # Find the nearest vertex
    nearest_vertex: Vertex = nearest_policy(new_sample, tree)

    # Steer to random position and get trajectory
    cost_to_new, traj_to_new = steering_policy(nearest_vertex, new_sample)

    # Check generated trajectory
    if collides(traj_to_new, env.obstacles):
        return False

    # Add new vertex to tree
    new_vertex = Vertex(
        position=traj_to_new.path[-1],
        parent=None,
        trajectory=traj_to_new,
        cost=cost_to_new,
    )

    # Get all vertices near new vertex
    near_vertices = find_near_vertices(tree, new_vertex, expand_dist=0.1, near_dist=2.0)
    # Add nearest by default to near vertices
    near_vertices.append(nearest_vertex)

    # Find the most optimal parent for new vertex
    cost, optimal_vertex = find_optimal_parent(
        new_vertex, near_vertices, steering_policy, env.obstacles
    )

    if not optimal_vertex:
        cost = cost_to_new
        optimal_vertex = nearest_vertex
    else:
        near_vertices.remove(optimal_vertex)

    # Connect new vertex with its parent
    new_vertex.parent = optimal_vertex

    # Rewire if required
    rewire(tree, new_vertex, near_vertices, steering_policy, env.obstacles)

    tree.vertices.append(new_vertex)

    # Check if goal is reached
    if np.linalg.norm(new_vertex.position.to_array() - goal) < params.expand_dist:
        return True

    return False


def update_plot(
    env: Environment, start: Point2d, goal: Point2d, tree: Tree, axes: Axes
):
    # Plot start and goal
    axes.plot(start.x, start.y, "*")
    axes.plot(goal.x, goal.y, "*")

    # Plot obstacles
    for obstacle in env.obstacles:
        obstacle.plot(axes)

    # Plot graph using BFS
    for vertex in tree.vertices:
        if vertex.parent:
            plt.plot(
                [point.x for point in vertex.trajectory.path],
                [point.y for point in vertex.trajectory.path],
                "b-",
            )
            plt.plot(vertex.position.x, vertex.position.y, "*b")

    current = tree.vertices[-1]
    while current.parent:
        plt.plot(
            [point.x for point in current.trajectory.path],
            [point.y for point in current.trajectory.path],
            "r-",
        )
        plt.plot(current.position.x, current.position.y, "*r")

        current = current.parent


def main():
    # Initialize environment
    planification_zone = Zone2d(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0)
    obstacles = [
        RectangleObstacle(pos=Point2d(x=5.0, y=4.0), width=1.0, height=1.0),
        RectangleObstacle(pos=Point2d(x=6.5, y=5.5), width=1.0, height=1.0),
    ]
    environment = Environment(
        planification_zone=planification_zone, obstacles=obstacles
    )

    # Set start and goal position
    start = Point2d(x=2.0, y=2.0)
    goal = Point2d(x=8.0, y=6.0)

    # Set parameters
    parameters = Parameters(
        max_nb_iterations=5000,
        expand_dist=0.2,
        goal_sample_rate=20,
        path_sampling_step=0.05,
        time_to_steer=0.05,
        velocity=1.0,
    )

    tree = Tree(vertices=[Vertex(position=start, parent=None, trajectory=[], cost=0.0)])

    plt.figure()
    axes = plt.subplot()

    def steering_policy(nearest, random):
        return constant_speed_line_steering_policy(
            nearest, random, parameters.time_to_steer, parameters.velocity
        )

    def nearest_policy(new, tree):
        return compute_nearest_euclidian_distance(new, tree)

    def sample_generation_policy():
        return generate_new_sample_biased_towards_goal(
            environment.planification_zone, goal, parameters.goal_sample_rate
        )

    for iteration in range(0, parameters.max_nb_iterations):
        if update(
            environment,
            parameters,
            goal,
            steering_policy,
            nearest_policy,
            sample_generation_policy,
            tree,
        ):
            break

    update_plot(environment, start, goal, tree, axes)

    plt.show()


if __name__ == "__main__":
    main()
