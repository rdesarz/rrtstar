import random

import numpy as np

from geometry import Zone2d, Point2d


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