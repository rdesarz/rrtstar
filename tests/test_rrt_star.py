import math
import unittest

from steering import constant_speed_line_steering_policy
from rrt_star import Vertex, rewire
from geometry import Point2d


class MyTestCase(unittest.TestCase):
    def test_constant_line_steering_policy(self):
        start = Vertex(position=Point2d(x=0.0, y=0.0), traj_to_vertex=None, parent=None, cost=1.0)
        dest = Point2d(x=3.0, y=0.0)
        time = 1.0
        velocity = 0.1

        cost, traj = constant_speed_line_steering_policy(start, dest, time, velocity)

        self.assertEqual(cost, 1.1)
        self.assertEqual(traj.path[0], start.position)
        self.assertEqual(traj.path[-1], Point2d(x=0.1, y=0.0))

    def test_rewire_rewiring_occurs(self):
        near_vertices = [
            Vertex(position=Point2d(x=0.0, y=0.0), traj_to_vertex=None, parent=None, cost=0.0),
            Vertex(position=Point2d(x=2.0, y=2.0), traj_to_vertex=None, parent=None, cost=math.sqrt(8)),
            Vertex(position=Point2d(x=3.0, y=1.0), traj_to_vertex=None, parent=None, cost=math.sqrt(8) + math.sqrt(2))
        ]

        near_vertices[1].parent = near_vertices[0]
        near_vertices[2].parent = near_vertices[1]

        new_vertex = Vertex(position=Point2d(x=0.0, y=0.0), traj_to_vertex=None, parent=near_vertices[2],
                            cost=near_vertices[2].cost + 0.5)

        def steering_policy(nearest, random):
            return constant_speed_line_steering_policy(
                nearest, random, time=1.0, velocity=1.0
            )

        rewire(new_vertex, near_vertices, steering_policy, [])

        self.assertEqual(near_vertices[2])

        if __name__ == "__main__":
            unittest.main()
