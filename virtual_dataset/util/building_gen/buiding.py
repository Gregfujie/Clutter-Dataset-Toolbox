from matplotlib import pyplot as plt


class Building:

    def __init__(self, data_path):
        self.fp_path = data_path
        self.walls = []
        self.doors = []
        self.model = []
        self.beams = []
        self.wall_thickness = 2
        self.x_offset = 0
        self.y_offset = 0
        self.height = 2.5
        self.size = [0, 0]

    def read_floorplan(self):
        f = open(self.fp_path, "r")
        lines = f.readlines()
        raws = []
        for line in lines:
            raw = line.split()
            raws.append(raw)
        if len(raws[0]) != 2:
            print("wrong shape")
            return False
        if len(raws[1]) != 1:
            print("wrong wall numbers")
            return False
        for i in range(2, len(raws)):
            if len(raws[i]) == 6:
                self.walls.append(raws[i])
            if raws[i][4] == "door":
                self.doors.append(raws[i])
        f.close()
        return True

    def create_model(self):
        # find reference
        # scaling
        # point and scale of wall
        # extend length according to thickness
        min_x, min_y, max_x, max_y = 1000, 1000, -1000, -1000
        door_beam = []
        h_walls, v_walls = [], []
        h_doors, v_doors = [], []
        for door in self.doors:
            point1 = [int(door[0]), int(door[1])]
            point2 = [int(door[2]), int(door[3])]
            if point1[0] == point2[0]:
                v_doors.append([point1[0], point1[1], point2[1]])
            elif point1[1] == point2[1]:
                h_doors.append([point1[1], point1[0], point2[0]])
        # v_doors.sort()
        # h_doors.sort()
        for wall in self.walls:
            point1 = [int(wall[0]), int(wall[1])]
            point2 = [int(wall[2]), int(wall[3])]
            max_x = max(max_x, max(point1[0], point2[0]))
            min_x = min(min_x, min(point1[0], point2[0]))
            max_y = max(max_y, max(point1[1], point2[1]))
            min_y = min(min_y, min(point1[1], point2[1]))
            if point1[0] == point2[0]:
                v_walls.append([point1[0], point1[1], point2[1], 1, 1])
            elif point1[1] == point2[1]:
                h_walls.append([point1[1], point1[0], point2[0], 1, 1])
        self.x_offset = (max_x - min_x) / 2 + min_x
        self.y_offset = (max_y - min_y) / 2 + min_y
        self.size[0] = abs(max_x - min_x)
        self.size[1] = abs(max_y - min_y)
        # v_walls.sort()
        # h_walls.sort()
        for each in v_doors:
            # find, pop, insert, add to beam
            for i in range(len(v_walls)):
                if v_walls[i][0] == each[0]:
                    if v_walls[i][1] < each[1] < each[2] < v_walls[i][2]:
                        new_wall_1 = [each[0], v_walls[i][1], each[1], v_walls[i][3], 0]
                        new_wall_2 = [each[0], each[2], v_walls[i][2], 0, v_walls[i][4]]
                        v_walls.pop(i)
                        v_walls.append(new_wall_1)
                        v_walls.append(new_wall_2)
                        door_beam.append([0, each[0], each[1], each[2]])
                        break
        for each in h_doors:
            # find, pop, insert, add to beam
            for i in range(len(h_walls)):
                if h_walls[i][0] == each[0]:
                    if h_walls[i][1] < each[1] < each[2] < h_walls[i][2]:
                        new_wall_1 = [each[0], h_walls[i][1], each[1], h_walls[i][3], 0]
                        new_wall_2 = [each[0], each[2], h_walls[i][2], 0, h_walls[i][4]]
                        h_walls.pop(i)
                        h_walls.append(new_wall_1)
                        h_walls.append(new_wall_2)
                        door_beam.append([1, each[0], each[1], each[2]])
                        break
        for wall in v_walls:
            wall[1] -= wall[3]*self.wall_thickness/2
            wall[2] += wall[4]*self.wall_thickness/2
            mid_p = [(wall[1] + wall[2]) / 2, wall[0]]
            x_len = self.wall_thickness
            y_len = abs(wall[2] - wall[1])
            self.model.append([mid_p, y_len, x_len, self.height])
        for wall in h_walls:
            wall[1] -= wall[3] * self.wall_thickness/2
            wall[2] += wall[4] * self.wall_thickness/2
            mid_p = [wall[0], (wall[1] + wall[2]) / 2]
            y_len = self.wall_thickness
            x_len = abs(wall[2] - wall[1])
            self.model.append([mid_p, y_len, x_len, self.height])
        for beam in door_beam:
            if beam[0] == 0:
                mid_p = [(beam[2] + beam[3]) / 2, beam[1]]
                x_len = self.wall_thickness
                y_len = abs(beam[3] - beam[2])
                self.beams.append([mid_p, y_len, x_len, self.height - 2])
            else:
                mid_p = [beam[1], (beam[2] + beam[3]) / 2]
                x_len = abs(beam[3] - beam[2])
                y_len = self.wall_thickness
                self.beams.append([mid_p, y_len, x_len, self.height - 2])
        times = self.wall_thickness / 0.2
        for each in self.model:
            each[0][0] -= self.y_offset
            each[0][0] /= times
            each[0][1] -= self.x_offset
            each[0][1] /= times
            each[1] /= times
            each[2] /= times
        for each in self.beams:
            each[0][0] -= self.y_offset
            each[0][0] /= times
            each[0][1] -= self.x_offset
            each[0][1] /= times
            each[1] /= times
            each[2] /= times


    def draw_walls(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for wall in self.walls:
            point = (int(wall[0]), int(wall[1]))
            width = int(wall[2]) - point[0]
            height = int(wall[3]) - point[1]
            if width == 0:
                width = 5
            elif height == 0:
                height = 5
            rect = plt.Rectangle(point, width, height)
            ax.add_patch(rect)
        plt.show()

    def get_dat_path(self):
        return self.fp_path

    def get_model(self):
        return self.model

    def get_beams(self):
        return self.beams


if __name__ == '__main__':
    fp_path = "floorplans/floorplan_0.txt"
    test = Building(fp_path)
    test.read_floorplan()
    test.draw_walls()
    print(test.get_dat_path())

    print("end")