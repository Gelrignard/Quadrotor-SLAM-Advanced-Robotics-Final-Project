from heapq import heappush, heappop  # Recommended.
import numpy as np
import math
from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

# compute distance, to get cost
def eu_dist( point1, point2, resolution):
    return math.sqrt(((point1[0] - point2[0])* resolution[0]) ** 2 + ((point1[1] - point2[1])* resolution[0]) ** 2 +((point1[2] - point2[2])* resolution[0]) ** 2)

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """
    astar = 0
    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    # Return a tuple (path, nodes_expanded)
    # shape of points
    shap = np.array(occ_map.map.shape)
    # distance for A*
    g = np.full(np.array(shap), np.inf)
    g[start_index] = 0
    # distance
    co = np.full(np.array(shap), np.inf)
    co[start_index] = 0
    # unknown parent
    p = np.full(shap, None)
    # initially open
    Q = np.zeros(shap, dtype=bool)
    # heap
    heap = []
    heappush(heap, (g[start_index], start_index))
    Num = 0
    direction = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (-1, 0, 0),
                 (0, -1, 0), (0, 0, -1), (-1, -1, 0), (1, 1, 0),
                 (1, 0, 1), (-1, 0, -1), (0, 1, 1), (0, -1, -1),
                 (1, -1, 0), (-1, 1, 0), (1, 0, -1), (-1, 0, 1),
                 (0, 1, -1), (0, -1, 1), (1, 1, 1), (-1, 1, 1),
                 (1, -1, 1), (-1, -1, 1), (1, 1, -1), (-1, 1, -1),
                 (1, -1, 1), (-1, -1, -1)]
    while(heap):
        # select
        c, u = heappop(heap)
        Q[u] = True
        Num += 1
        # true cost
        cu = co[u]
        for direct in direction:
            neib = (direct[0] + u[0], direct[1] + u[1], direct[2] + u[2])
            if occ_map.is_occupied_index(neib):
                continue
            if Q[neib]:
                continue
            cost = cu + eu_dist(neib, u, resolution)
            if cost < co[neib]:
                co[neib] = cost
                p[neib] = u
                if astar:
                    g[neib] = cost + eu_dist(neib, goal_index, resolution)
                else:
                    g[neib] = cost
                heappush(heap, (g[neib], neib))
            if goal_index == neib:
                path = []
                curr = goal_index
                path.insert(0,goal)
                while curr != start_index:
                    path.insert(0,occ_map.index_to_metric_center(curr))
                    curr = p[curr]
                path.insert(0, start)
                path = np.array(path)
                return path, Num
    return None, Num