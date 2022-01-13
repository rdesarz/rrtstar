import typing

import numpy as np

from geometry import Point2d, Trajectory
from rrt_star import Vertex


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