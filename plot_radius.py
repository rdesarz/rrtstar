import matplotlib.pyplot as plt
import math

n_vertices = range(1, 1000)
near_dist = 1.0
radiuses = [near_dist * math.sqrt(math.log(n) / n) for n in n_vertices]

plt.plot(n_vertices, radiuses)
plt.show()
