class Map2D(object):
    """Store the position of obstacles in a grid.
    Right now it can only load from a single file.
    The list of all obstacles is stored in self.obstacles
    and a grid with the obstacles near each part
    of the space is stored in self.grid.
    The obstacles are input through *data*.
    This can be either a list of points or
    a filename / file-object with the data in it.
    """
    def __init__(self, data, radius, x0=None, xf=None, y0=None, yf=None):
        self.minLx = x0
        self.maxLx = xf
        self.minLy = y0
        self.maxLy = yf
        self.obstacles = []
        self.grid_updated = False
        self.load_obstacles(data)
        nx = int(self.Lx / radius)
        ny = int(self.Ly / radius)
        self.setup_boxes(nx, ny)
        self.fill_grid()
        return

    def load_obstacles(self, obstacles):
        if type(obstacles) == str:
            self.add_from_name(obstacles)
        elif type(obstacles) == file:
            self.add_from_file(obstacles)
        elif type(obstacles) == list:
            self.add_from_list(obstacles)
        else:
            raise Exception("Map2D: unknown data type {:}".format(obstacles))
        return

    def add_from_name(self, filename):
        """Read a collection of *x y* pairs
        from the file *filename*.
        The dimensions of the available space
        are deduced from these points.
        """
        with open(filename, 'r') as f:
            self.add_from_file(f)
        return

    def add_from_file(self, f):
        """Read a collection of *x y* pairs
        from the file object *f*.
        The dimensions of the available space
        are deduced from these points.
        """
        obstacles = []
        while True:
            try:
                x, y = [float(o) for o in f.readline().split()]
                obstacles.append([x, y])
            except:
                break
        self.add_from_list(obstacles)
        return

    def add_from_list(self, obstacles):
        """Load the obstacles from *obstacles*."""
        xs = ([o[0] for o in obstacles])
        ys = ([o[1] for o in obstacles])

        # Determine the limits of the box
        if self.minLx is None:
            self.minLx = min(xs) - 0.01 * abs(min(xs))
        if self.maxLx is None:
            self.maxLx = max(xs) + 0.01 * abs(max(xs))
        self.Lx = self.maxLx - self.minLx
        if self.minLy is None:
            self.minLy = min(ys) - 0.01 * abs(min(ys))
        if self.maxLy is None:
            self.maxLy = max(ys) + 0.01 * abs(max(ys))
        self.Ly = self.maxLy - self.minLy

        self.obstacles.extend(obstacles)
        self.grid_updated = False
        return

    def setup_boxes(self, nx, ny):
        """ Define how many boxes per axis the grid will have.
        Total number of boxes = nx * ny.
        nx = number of boxes along x. (int > 3)
        ny = number of boxes along y. (int > 3)
        """
        self.nx = int(nx)
        self.ny = int(ny)
        self.grid = [[] for i in range(nx * ny)]  # initialize empty grid
        return

    def which_box(self, pos):
        """ Gives the (i,j) indexes corresponding to position pos.
        """
        i = int((pos[0] - self.minLx) * self.nx / self.Lx)
        j = int((pos[1] - self.minLy) * self.ny / self.Ly)
        assert i >= 0 and i < self.nx
        assert j >= 0 and j < self.ny
        return (i, j)

    def obstacles_in_box(self, i, j):
        """Returns a list with the agents in a given box.
        Periodic boundaries implemented, so if i (j) is larger than
        nx (ny) it is replaced by i%nx (j%ny).
        i = inner-most index of the box. (int)
        j = outer-most index of the box. (int)
        """
        i, j = i % self.nx, j % self.ny
        return self.grid[i + j * self.nx]

    def obstacles_near(self, pos):
        i, j = self.which_box(pos)
        return self.obstacles_in_box(i, j)

    def fill_grid(self):
        """ Fill the grid[] with a list of the obstacles contained
        in each element. If obstacle is stored in its box + the 8
        surrounding ones, so that a robot that senses obstacles in
        its box will have information of its surrounding.
        """
        nx, ny = self.nx, self.ny
        for i in range(nx * ny):
            self.grid[i] = []
        for o in self.obstacles:
            i, j = self.which_box(o)
            for dj in (-1, 0, 1):
                jj = j + dj
                if jj >= 0 and jj < ny:
                    for di in (-1, 0, 1):
                        ii = i + di
                        if ii >= 0 and ii < nx:
                            self.grid[ii + jj * nx].append(o)
        self.grid_updated = True
        return self.grid

    def filtered_map(self, threshold=1):
        """For each grid box that contains
        at least *treshold* obstacles,
        return their mean position.
        """
        fmap = []
        if not self.grid_updated:
            self.fill_grid()
        for i in range(self.nx * self.ny):
            obs = self.grid[i]
            if len(obs) >= threshold:
                x = sum(o[0] for o in obs) / len(obs)
                y = sum(o[1] for o in obs) / len(obs)
                fmap.append([x, y])
        return fmap
