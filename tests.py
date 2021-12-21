import unittest

import main


class MyTestCase(unittest.TestCase):
    def test_generate_new_sample(self):
        point = main.generate_new_sample(main.Zone2d(x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0))
        print(point)

    def test_compute_nearest(self):
        graph = main.Graph(vertices=[main.Vertex(position=main.Point2d(x=2.0, y=2.0), connected=[]),
                                     main.Vertex(position=main.Point2d(x=3.0, y=3.0), connected=[])])
        point = main.Point2d(x=3.0, y=3.0)

        nearest = main.compute_nearest(point, graph)

        self.assertEqual(nearest, graph.vertices[1].position)


if __name__ == '__main__':
    unittest.main()
