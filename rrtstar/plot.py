from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Circle

from rrtstar.geometry import Point2d
from rrtstar.params import Environment, Parameters
from rrtstar.rrt_star import Tree, Vertex


def update_plot(
        env: Environment, params: Parameters, start: Point2d, goal: Point2d, tree: Tree, axes: Axes,
        nodes_to_goal: Vertex
):
    # Plot start and goal
    axes.plot(start.x, start.y, "or")
    axes.add_patch(Circle((goal.x, goal.y), params.goal_zone_radius))

    # Plot obstacles
    for obstacle in env.obstacles:
        obstacle.plot(axes)

    # Plot graph using BFS
    for vertex in tree.vertices:
        if vertex.parent:
            plt.plot(
                [point.x for point in vertex.traj_to_vertex.path],
                [point.y for point in vertex.traj_to_vertex.path],
                color="saddlebrown",
            )
            plt.plot(vertex.position.x, vertex.position.y, markersize="3", marker="o", color="olive")

    if nodes_to_goal:
        current = nodes_to_goal
        while current.parent:
            plt.plot(
                [point.x for point in current.traj_to_vertex.path],
                [point.y for point in current.traj_to_vertex.path],
                "r-",
            )
            plt.plot(current.position.x, current.position.y, markersize="3", marker="o", color="r")

            current = current.parent
