import numpy as np

from .graph_search import graph_search
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import inv
class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.6

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1,3)) # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        # velocity
        v = 2.92
        # ramer douglas peucker algorithm, initial point set
        point = self.ramer_douglas_peucker(self.path, 0.25)
        inipath = point
        i = 0
        # while(np.linalg.norm(point[-1] == inipath[i])<=1):
        #     dist = np.linalg.norm(point[i+1] - point[i])
        #     if dist < 1:
        #         continue
        #     else:
        #         avr = (point[i+1] + point[i])/2
        #         inipath = np.insert(inipath, i+1, avr, axis = 0)
        #     i += 1
        i = 0
        # averages = (point[:-1] + point[1:]) / 2
        # inipath = np.insert(point, np.arange(1, len(point)), averages, axis=0)
        inipath = self.insert_midpoints_path(point) #TODO
        inipath = self.insert_midpoints_path(inipath) #TODO
        inipath = self.insert_midpoints_path(inipath) #TODO
        inipath = self.insert_midpoints_path(inipath) #TODO
        # inipath = self.insert_midpoints_path(inipath)
        # inipath = self.insert_midpoints_path(inipath)
        # averages = (inipath[:-1] + inipath[1:]) / 2
        # inipath = np.insert(inipath, np.arange(1, len(inipath)), averages, axis=0)
        # averages = (inipath[:-1] + inipath[1:]) / 2
        # inipath = np.insert(inipath, np.arange(1, len(inipath)), averages, axis=0)
        # inipath = self.path
        # calculate norm, time
        inidis = np.linalg.norm(inipath[:-1] - inipath[1:], axis = 1)
        # print(inidis)
        tim = np.cumsum(inidis)/v
        # print(tim)
        # get matrix A, B for x, y, z seperately
        n = np.shape(tim)[0]
        # print(n)
        # print(inipath.shape)
        '''
        ma = np.array([
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 2, 0, 0]
        ])
        Bx = np.array([[start[0]], [0], [0]])
        By = np.array([[start[1]], [0], [0]])
        Bz = np.array([[start[2]], [0], [0]])
        A = np.block([ma, np.zeros((3, 6*(n - 1)))])
        for i in range(n-1):
            x1 = inipath[i+1, 0]
            y1 = inipath[i+1, 1]
            z1 = inipath[i+1, 2]
            # print("...")
            # print(x1,y1,z1)
            # t = tim[i]
            # ma = np.array([
            #     [t**5, t**4, t**3, t**2, t, 1, 0, 0, 0, 0, 0, 0],
            #     [0, 0, 0, 0, 0, 0, t**5, t**4, t**3, t**2, t, 1],
            #     [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0, -5*t**4, -4*t**3, -3*t**2, -2*t, -1, 0],
            #     [20*t**3, 12*t**2, 6*t, 2, 0, 0, -20*t**3, -12*t**2, -6*t, -2, 0, 0],
            #     [60*t**2, 24*t, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            #     [0, 0, 0, 0, 0, 0, 60*t**2, 24*t, 6, 0, 0, 0]
            # ])
            t = inidis[i]/v
            # ma = np.array([
            #     [t**5, t**4, t**3, t**2, t, 1, 0, 0, 0, 0, 0, 0],
            #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            #     [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0, 0, 0, 0, 0, -1, 0],
            #     [20*t**3, 12*t**2, 6*t, 2, 0, 0, 0, 0, 0, -2, 0, 0],
            #     [60*t**2, 24*t, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            #     [0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0]
            # ])
            ma = np.array([
                [t ** 5, t ** 4, t ** 3, t ** 2, t, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                [-5 * t ** 4, -4 * t ** 3, -3 * t ** 2, -2 * t, -1, 0, 0, 0, 0, 0, 1, 0],
                [-20 * t ** 3, -12 * t ** 2, -6 * t, -2, 0, 0, 0, 0, 0, 2, 0, 0],
                [-60 * t ** 2, -24 * t, -6, 0, 0, 0, 0, 0, 6, 0, 0, 0],
                [-120 * t, -24, 0, 0, 0, 0, 0, 24, 0, 0, 0, 0]
            ])
            A = np.block([[A], [np.zeros((6, 6*i)), ma, np.zeros((6, 6*(n - 2 - i)))]])
            Bx = np.block([[Bx], [x1], [x1], [0], [0], [0], [0]])
            By = np.block([[By], [y1], [y1], [0], [0], [0], [0]])
            Bz = np.block([[Bz], [z1], [z1], [0], [0], [0], [0]])
        # t = tim[n-1]
        t = inidis[-1] / v
        x1 = goal[0]
        y1 = goal[1]
        z1 = goal[2]
        ma = np.array([
            [t**5, t**4, t**3, t**2, t, 1],
            [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0],
            [20*t**3, 12*t**2, 6*t, 2, 0, 0]
        ])
        A = np.block([[A], [np.zeros((3, 6*(n - 1))), ma]])
        Bx = np.block([[Bx], [x1], [0], [0]]).reshape(6*n)
        By = np.block([[By], [y1], [0], [0]]).reshape(6*n)
        Bz = np.block([[Bz], [z1], [0], [0]]).reshape(6*n)
        # print(Bz)
        '''
        # constrains
        A = np.zeros((6*n, 6*n))
        Bx = np.zeros(6*n)
        By = np.zeros(6*n)
        Bz = np.zeros(6*n)
        '''
        # start
        A[0, 5] = 1
        A[1, 4] = 1
        A[2, 3] = 2
        Bx[0] = start[0]
        By[0] = start[1]
        Bz[0] = start[2]
        # im = (point[1] - start)/1000
        # Bx[1] = im[0]
        # By[1] = im[1]
        # Bz[1] = im[2]
        # end
        t = inidis[-1] / v
        Bx[6*n-3] = goal[0]
        By[6*n-3] = goal[1]
        Bz[6*n-3] = goal[2]
        A[6*n-3:6*n, 6*n-6:6*n] = np.array([
            [t ** 5, t ** 4, t ** 3, t ** 2, t, 1],
            [5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1, 0],
            [20 * t ** 3, 12 * t ** 2, 6 * t, 2, 0, 0]
        ])
        for i in range(n-1):
            t = inidis[i]/v
            # end segment pos
            A[i+3, 6*i:6*i+6] = np.array([t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
            Bx[i+3] = inipath[i, 0]
            By[i+3] = inipath[i, 1]
            Bz[i+3] = inipath[i, 2]
            # start segment pos
            A[i+n+2, 6*i + 11] = 1
            Bx[i+n+2] = inipath[i, 0]
            By[i+n+2] = inipath[i, 1]
            Bz[i+n+2] = inipath[i, 2]
            # vel continue
            A[i + 2*n + 1, 6*i: 6*i+12] = np.array([5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1, 0, 0, 0, 0, 0, -1, 0])
            # acc cont
            A[i + 3*n, 6*i: 6*i+12] = np.array([20 * t ** 3, 12 * t ** 2, 6 * t, 2, 0, 0, 0, 0, 0, -2, 0, 0])
            # ... 0
            # A[i + 4 * n - 1, 6 * i: 6 * i + 6] = np.array([60 * t ** 2, 24 * t, 6, 0, 0, 0])
            # A[i + 5*n -2 , 6*i + 6: 6*i+12] = np.array([0, 0, 0, 6, 0, 0])
            # ... cont
            A[i + 4*n -1 , 6*i: 6*i+12] = np.array([60 * t ** 2, 24 * t, 6, 0, 0, 0, 0, 0, -6, 0, 0, 0])
            A[i + 5*n -2 , 6*i: 6*i+12] = np.array([120 * t, 24, 0, 0, 0, 0, 0, -24, 0, 0, 0, 0])
        '''
        for i in range(n-1):
            t = inidis[i]/v
            # end segment pos
            A[i, 6*i:6*i+6] = np.array([t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
            Bx[i] = inipath[i+1, 0]
            By[i] = inipath[i+1, 1]
            Bz[i] = inipath[i+1, 2]
            # start segment pos
            A[i+n, 6*i + 5] = 1
            Bx[i+n] = inipath[i, 0]
            By[i+n] = inipath[i, 1]
            Bz[i+n] = inipath[i, 2]
            # vel continue
            A[i + 2*n, 6*i: 6*i+12] = np.array([5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1, 0, 0, 0, 0, 0, -1, 0])
            # acc cont
            A[i + 3*n, 6*i: 6*i+12] = np.array([20 * t ** 3, 12 * t ** 2, 6 * t, 2, 0, 0, 0, 0, 0, -2, 0, 0])
            # ... 0
            # A[i + 4 * n - 1, 6 * i: 6 * i + 6] = np.array([60 * t ** 2, 24 * t, 6, 0, 0, 0])
            # A[i + 5*n -2 , 6*i + 6: 6*i+12] = np.array([0, 0, 0, 6, 0, 0])
            # ... cont
            A[i + 4*n, 6*i: 6*i+12] = np.array([60 * t ** 2, 24 * t, 6, 0, 0, 0, 0, 0, -6, 0, 0, 0])
            A[i + 5*n, 6*i: 6*i+12] = np.array([120 * t, 24, 0, 0, 0, 0, 0, -24, 0, 0, 0, 0])
        i = n-1
        t = inidis[i] / v
        # end pos
        A[i, 6 * i:6 * i + 6] = np.array([t ** 5, t ** 4, t ** 3, t ** 2, t, 1])
        Bx[i] = inipath[i + 1, 0]
        By[i] = inipath[i + 1, 1]
        Bz[i] = inipath[i + 1, 2]
        # start segment pos
        A[i + n, 6 * i + 5] = 1
        Bx[i + n] = inipath[i, 0]
        By[i + n] = inipath[i, 1]
        Bz[i + n] = inipath[i, 2]
        # vel end
        A[i + 2 * n, 6 * i: 6 * i + 6] = np.array([5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1, 0])
        # acc end
        A[i + 3 * n, 6 * i: 6 * i + 6] = np.array([20 * t ** 3, 12 * t ** 2, 6 * t, 2, 0, 0])
        # start vel, acc
        A[i + 4 * n, 4] = 1
        A[i + 5 * n, 3] = 2

        # solve the polynomial for each segment
        self.cnt = n
        '''
        inA = np.linalg.inv(A)
        self.cx = (inA@Bx).reshape(n, 6)
        print(self.cx)
        self.cy = (inA@By).reshape(n, 6)
        self.cz = (inA@Bz).reshape(n, 6)
        '''
        self.cx = np.linalg.solve(A, Bx).reshape(n, 6)
        self.cy = np.linalg.solve(A, By).reshape(n, 6)
        self.cz = np.linalg.solve(A, Bz).reshape(n, 6)
        self.points = inipath
        self.time = np.insert(tim, 0, 0)
        # self.time = tim
        self.v = v
        # print(self.points)
        if self.points.shape[0] == 1:
            self.flag = True
        else:
            self.flag = False
        # calc path
        #     # time
        #     dist = np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))
        #     self.time = dist / self.v  # N-1 numbers


    # auxiliary function for ramer douglas peucker algorithm
    def perpendicular_distance(self, point, starts, ends):
        if np.all(starts == ends):
            return np.linalg.norm(point - starts)
        return np.divide(np.abs(np.linalg.norm(np.cross(ends - starts, starts - point))), np.linalg.norm(ends - starts))

    # ramer douglas peucker algorithm
    def ramer_douglas_peucker(self, points, epsilon):
        # Find the point with the maximum distance from the line formed by the first and last points
        dmax = 0.0
        index = 0
        for i in range(1, len(points) - 1):
            d = self.perpendicular_distance(points[i], points[0], points[-1])
            if d > dmax:
                index = i
                dmax = d
        # If max distance is greater than epsilon, recursively simplify
        if dmax > epsilon:
            # Recursive call
            rec_results1 = self.ramer_douglas_peucker(points[:index + 1], epsilon)
            rec_results2 = self.ramer_douglas_peucker(points[index:], epsilon)
            # Build the final list
            result = np.vstack((rec_results1[:-1], rec_results2))
        else:
            result = np.vstack((points[0], points[-1]))
        return result

    def insert_midpoints_path(self, points):
        new_points = [points[0].tolist()]
        for i in range(1, len(points)):
            if np.linalg.norm(points[i] - points[i - 1]) > 2:
                midpoint = (points[i] + points[i - 1]) / 2
                new_points.append(midpoint.tolist())
            new_points.append(points[i].tolist())

        return np.array(new_points)



    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        # if self.flag:
        #     x = self.points[0]
        # elif t < 0:
        #     x = self.points[0]
        # elif t >= self.time[-1]:
        #     x = self.points[-1]
        # else:
        # print("**")
        # t = self.time[1]
        # print(t)
        if t <= 0:
            x = self.points[0]
        elif t >= self.time[-1]:
            x = self.points[-1]
        else:
            num = np.searchsorted(self.time, t)
            # if num == self.cnt:
            #     num -= 1
            t0 = self.time[num-1]
            t = t - t0
            # cxs = np.array([self.cx[6*num], self.cx[6*num]])
            cxs = self.cx[num-1]
            x[0] = cxs[0]*t**5 + cxs[1]*t**4 + cxs[2]*t**3 + cxs[3]*t**2 + cxs[4]*t + cxs[5]
            x_dot[0] = 5*cxs[0]*t**4 + 4*cxs[1]*t**3 + 3*cxs[2]*t**2 + 2*cxs[3]*t + cxs[4]
            cys = self.cy[num-1]
            x[1] = cys[0]*t**5 + cys[1]*t**4 + cys[2]*t**3 + cys[3]*t**2 + cys[4]*t + cys[5]
            x_dot[1] = 5*cys[0]*t**4 + 4*cys[1]*t**3 + 3*cys[2]*t**2 + 2*cys[3]*t + cys[4]
            czs = self.cz[num-1]
            x[2] = czs[0]*t**5 + czs[1]*t**4 + czs[2]*t**3 + czs[3]*t**2 + czs[4]*t + czs[5]
            x_dot[2] = 5*czs[0]*t**4 + 4*czs[1]*t**3 + 3*czs[2]*t**2 + 2*czs[3]*t + czs[4]
            # print(czs)
        # print(x_dot)

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
