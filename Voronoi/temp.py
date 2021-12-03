from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import matplotlib.pyplot as plt

points = np.array([[1, 5], [3, 4], [-2, 1], [4, -2]])
vor = Voronoi(points)
voronoi_plot_2d(vor)
plt.show()