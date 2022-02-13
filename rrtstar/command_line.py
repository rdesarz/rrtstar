from matplotlib import pyplot as plt

from rrtstar.geometry import Zone2d, Point2d
from rrtstar.nearest import compute_nearest_euclidian_distance
from rrtstar.params import Environment, Parameters
from rrtstar.plot import update_plot
from rrtstar.rrt_star import Tree, Vertex, update_tree
from rrtstar.sampling import generate_new_sample_uniform
from rrtstar.steering import constant_speed_line_steering_policy


def main():
    # Initialize environment
    planification_zone = Zone2d(x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0)
    obstacles = []
    environment = Environment(
        planification_zone=planification_zone, obstacles=obstacles
    )

    # Set start and goal position
    start = Point2d(x=0.0, y=0.0)
    goal = Point2d(x=8.0, y=6.0)

    # Set parameters
    parameters = Parameters(
        max_nb_iterations=500,
        expand_dist=0.2,
        goal_sample_rate=20,
        path_sampling_step=0.05,
        time_to_steer=1.0,
        velocity=15.0,
        near_dist=10.0
    )

    tree = Tree(vertices=[Vertex(position=start, parent=None, traj_to_vertex=None, cost=0.0)])

    plt.figure()
    axes = plt.subplot()

    def steering_policy(nearest, random):
        return constant_speed_line_steering_policy(
            nearest, random, parameters.time_to_steer, parameters.velocity
        )

    def nearest_policy(new, tree):
        return compute_nearest_euclidian_distance(new, tree)

    def sample_generation_policy():
        return generate_new_sample_uniform(
            environment.planification_zone
        )

    node_to_goal = None
    for iteration in range(0, parameters.max_nb_iterations):
        result = update_tree(
            environment,
            parameters,
            goal,
            steering_policy,
            nearest_policy,
            sample_generation_policy,
            tree,
        )

        if result:
            node_to_goal = result

    update_plot(environment, parameters, start, goal, tree, axes, node_to_goal)

    plt.show()
