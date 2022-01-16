import numpy as np

from rrtstar.geometry import Point2d
from rrtstar.rrt_star import Tree, Vertex


def compute_nearest_euclidian_distance(new_sample: Point2d, graph: Tree) -> Vertex:
    norms = [
        np.linalg.norm(vertex.position.to_array() - new_sample.to_array())
        for vertex in graph.vertices
    ]

    return graph.vertices[norms.index(min(norms))]
