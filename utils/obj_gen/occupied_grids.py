from math import floor


def calculate_grids(input_x, input_y):
    grids = []
    x_min = round(min(input_x) - 0.2, 1)
    x_max = round(max(input_x) + 0.2, 1)
    y_min = round(min(input_y) - 0.2, 1)
    y_max = round(max(input_y) + 0.2, 1)
    cellsize = 0.5
    length = floor((x_max - x_min) / cellsize)
    width = floor((y_max - y_min) / cellsize)
    for each_x, each_y in zip(input_x, input_y):
        cur_x = round(each_x, 1)
        cur_y = round(each_y, 1)
        n_l = int((cur_x - x_min) / cellsize)
        n_w = int((cur_y - y_min) / cellsize)
        cur_cell = [[round(x_min + cellsize * n_l, 1), round(y_min + cellsize * n_w, 1)],
                    [round(x_min + cellsize * (n_l + 1), 1), round(y_min + cellsize * n_w, 1)],
                    [round(x_min + cellsize * n_l, 1), round(y_min + cellsize * (n_w + 1), 1)],
                    [round(x_min + cellsize * (n_l + 1), 1), round(y_min + cellsize * (n_w + 1), 1)]]
        if cur_cell not in grids:
            grids.append(cur_cell)
    return grids