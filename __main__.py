import matplotlib.pyplot as plt

from geometry import Point2d, Zone2d, RectangleObstacle
from nearest import compute_nearest_euclidian_distance
from params import Parameters, Environment
from plot import update_plot
from rrt_star import update_tree, Vertex, Tree
from sampling import generate_new_sample_uniform
from steering import constant_speed_line_steering_policy


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
        max_nb_iterations=10000,
        expand_dist=0.2,
        goal_sample_rate=20,
        path_sampling_step=0.05,
        time_to_steer=1.0,
        velocity=15.0,
        goal_zone_radius=0.05,
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

    for iteration in range(0, parameters.max_nb_iterations):
        if update_tree(
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
