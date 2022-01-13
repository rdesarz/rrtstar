import unittest

from steering import constant_speed_line_steering_policy
from rrt_star import Vertex
from geometry import Point2d


class MyTestCase(unittest.TestCase):
    def test_constant_line_steering_policy(self):
        start = Vertex(position=Point2d(x=0.0, y=0.0), trajectory=[], parent=None, cost=1.0)
        dest = Point2d(x=3.0, y=0.0)
        time = 1.0
        velocity = 0.1

        cost, traj = constant_speed_line_steering_policy(start, dest, time, velocity)

        self.assertEqual(cost, 1.1)
        self.assertEqual(traj.path[0], start.position)
        self.assertEqual(traj.path[-1], Point2d(x=0.1, y=0.0))



if __name__ == "__main__":
    unittest.main()
