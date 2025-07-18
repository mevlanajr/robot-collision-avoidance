import toppra.algorithm.algorithm as algorithm
import numpy as np


class ConvexOptAlgorithm(algorithm.ParameterizationAlgorithm):
    def __init__(self, constraint_list, path, gridpoints=None):
        super(ConvexOptAlgorithm, self).__init__(constraint_list, path, gridpoints)
        self.end_path_velocities = np.array([0.0, 0.0])

    def compute_parameterization(self):
        """Return a time-scale parameterization."""
        pass
