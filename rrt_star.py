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
    traj_to_vertex: typing.Optional[Trajectory]
    parent: typing.Optional["Vertex"] = None
    cost: float = 0.0


@dataclass
class Tree:
    vertices: typing.List[Vertex]


def find_near_vertices(tree: Tree, new_vertex: Vertex, near_dist: float, reachable_dist: typing.Optional[float]) -> \
        typing.List[Vertex]:
    n_vertices = len(tree.vertices)
    dist_range = near_dist * math.sqrt(math.log(n_vertices) / n_vertices)

    if reachable_dist:
        dist_range = min(reachable_dist, dist_range)

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
) -> typing.Tuple[float, typing.Optional[Vertex], Trajectory]:
    cost_min = sys.float_info.max
    vertex_min = None
    traj_to_min = None
    for near_vertex in near_vertices:
        cost_to_new, traj_to_new = steering_policy(near_vertex, new_vertex.position)
        if (
                not collides(trajectory=traj_to_new, obstacles=obstacles)
                and traj_to_new.path[-1] == new_vertex.position
        ):
            if near_vertex.cost + cost_to_new < cost_min:
                cost_min = near_vertex.cost + cost_to_new
                vertex_min = near_vertex
                traj_to_min = traj_to_new

    return cost_min, vertex_min, traj_to_min


def propagate_cost_to_leaves(parent: Vertex, tree: Tree):
    for vertex in tree.vertices:
        if parent and vertex.parent and parent.position == vertex.parent.position:
            vertex.cost = parent.cost + np.linalg.norm(parent.position.to_array() - vertex.position.to_array())
            propagate_cost_to_leaves(vertex, tree)


def rewire(tree: Tree, new_vertex: Vertex, near_vertices: typing.List[Vertex], steering_policy,
           obstacles: typing.List[RectangleObstacle]):
    for near_vertex in near_vertices:
        cost_to_near, traj_to_near = steering_policy(new_vertex, near_vertex.position)
        if (
                not collides(trajectory=traj_to_near, obstacles=obstacles)
                and traj_to_near.path[-1] == near_vertex.position
                and new_vertex.cost + cost_to_near < near_vertex.cost
        ):
            near_vertex.parent = new_vertex
            near_vertex.cost = new_vertex.cost + cost_to_near
            near_vertex.traj_to_vertex = traj_to_near
            propagate_cost_to_leaves(near_vertex, tree)


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
        traj_to_vertex=traj_to_new,
        cost=nearest_vertex.cost + cost_to_new,
    )

    # Get all vertices near new vertex
    near_vertices = find_near_vertices(tree, new_vertex, params.near_dist,
                                       reachable_dist=params.velocity * params.time_to_steer)

    # Find the most optimal parent for new vertex
    optimal_cost, optimal_vertex, optimal_traj = find_optimal_parent(
        new_vertex, near_vertices, steering_policy, env.obstacles
    )

    # If no optimal vertex is found, use the nearest one by default. If it is found, remove it
    if not optimal_vertex:
        optimal_cost = nearest_vertex.cost + cost_to_new
        optimal_vertex = nearest_vertex
        optimal_traj = traj_to_new
    else:
        near_vertices.remove(optimal_vertex)

    # Connect new vertex with its parent
    new_vertex.parent = optimal_vertex
    new_vertex.cost = optimal_cost
    new_vertex.traj_to_vertex = optimal_traj

    # Rewire if required
    rewire(tree, new_vertex, near_vertices, steering_policy, env.obstacles)

    # Add new vertex to tree
    tree.vertices.append(new_vertex)

    # Check if goal is reached
    if np.linalg.norm(new_vertex.position.to_array() - goal) < params.goal_zone_radius:
        return True

    return False
