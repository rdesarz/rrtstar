import typing

from geometry import Zone2d, RectangleObstacle


class Parameters(typing.NamedTuple):
    max_nb_iterations: int
    expand_dist: float
    goal_sample_rate: int
    path_sampling_step: float
    time_to_steer: float
    velocity: float
    near_dist: float
    goal_zone_radius: typing.Optional[float] = None


class Environment(typing.NamedTuple):
    planification_zone: Zone2d
    obstacles: typing.List[RectangleObstacle]
