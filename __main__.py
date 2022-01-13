import matplotlib.pyplot as plt

from geometry import Point2d, Zone2d, RectangleObstacle
from nearest import compute_nearest_euclidian_distance
from params import Parameters, Environment
from plot import update_plot
from rrt_star import update_tree, Vertex, Tree
from sampling import generate_new_sample_biased_towards_goal
from steering import constant_speed_line_steering_policy


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
