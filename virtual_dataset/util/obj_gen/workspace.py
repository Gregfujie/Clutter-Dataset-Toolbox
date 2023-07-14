import numpy as np
import math


class WorkSpace:

    def __init__(self):
        self.walls_to_cut = []
        self.x_min = 100
        self.x_max = -100
        self.y_min = 100
        self.y_max = -100
        self.z_min = 100
        self.z_max = -100

    def add_wall(self, position, scale, yaw):
        x, y, z = position
        delta_x = np.array([[scale[0] / 2], [0]])
        delta_y = np.array([[0], [scale[1] / 2]])
        trans = np.array([[math.cos(yaw), math.sin(yaw)],
                          [-math.sin(yaw), math.cos(yaw)]])
        result1 = trans @ delta_x
        result2 = trans @ delta_y
        self.walls_to_cut.append([[x - result1[0][0] - result2[0][0], y - result1[1][0] - result2[1][0]],
                             [x + result1[0][0] - result2[0][0], y + result1[1][0] - result2[1][0]],
                             [x - result1[0][0] + result2[0][0], y - result1[1][0] + result2[1][0]],
                             [x + result1[0][0] + result2[0][0], y + result1[1][0] + result2[1][0]]])

        self.x_min = min(self.x_min, x)
        self.x_max = max(self.x_max, x)
        self.y_min = min(self.y_min, y)
        self.y_max = max(self.y_max, y)
        self.z_min = min(self.z_min, z)
        self.z_max = max(self.z_max, z)

    def get_range(self):
        return [self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max]

    def get_cuts(self):
        return self.walls_to_cut


if __name__ == '__main__':
    print("end")