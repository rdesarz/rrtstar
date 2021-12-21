import unittest

from main import generate_new_sample, Zone2d


class MyTestCase(unittest.TestCase):
    def test_generate_new_sample(self):
        point = generate_new_sample(Zone2d(x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0))
        print(point)


if __name__ == '__main__':
    unittest.main()
