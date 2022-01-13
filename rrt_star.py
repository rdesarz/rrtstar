import math
import sys
import typing

import numpy as np
from dataclasses import dataclass

from params import Parameters, Environment
from geometry import Point2d, RectangleObstacle, Trajectory, collides


@dataclass
class Vertex:
    position: Point2d
    trajectory: Trajectory
    parent: typing.Optional["Vertex"] = None
    cost: float = 0.0


@dataclass
class Tree:
    vertices: typing.List[Vertex]


def find_near_vertices(tree: Tree, new_vertex: Vertex, near_dist: float) -> typing.List[Vertex]:
    n_vertices = len(tree.vertices)
    dist_range = near_dist * math.sqrt(math.log(n_vertices) / n_vertices)

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


def update_tree(
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
    near_vertices = find_near_vertices(tree, new_vertex, near_dist=2.0)
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