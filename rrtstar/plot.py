from matplotlib import pyplot as plt
from matplotlib.axes import Axes

from rrtstar.geometry import Point2d
from rrtstar.params import Environment
from rrtstar.rrt_star import Tree


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
                [point.x for point in vertex.traj_to_vertex.path],
                [point.y for point in vertex.traj_to_vertex.path],
                "b-",
            )
            plt.plot(vertex.position.x, vertex.position.y, "*b")

    current = tree.vertices[-1]
    while current.parent:
        plt.plot(
            [point.x for point in current.traj_to_vertex.path],
            [point.y for point in current.traj_to_vertex.path],
            "r-",
        )
        plt.plot(current.position.x, current.position.y, "*r")

        current = current.parent