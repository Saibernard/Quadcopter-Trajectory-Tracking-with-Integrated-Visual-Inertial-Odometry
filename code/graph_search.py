from heapq import heappush, heappop  # Recommended.
import numpy as np
from collections import defaultdict

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

def find_near_cubes(index_value, graph_shape, occ_map):
    graph_shape = np.array([graph_shape])

    possible_outputs = np.array([[i, j, k] for i in [-1, 0, 1] for j in [-1, 0, 1] for k in [-1, 0, 1]])

    # neighbors = np.clip(index_value + possible_outputs, 0, np.array(graph_shape) - 1)
    # x, y, z = neighbors[:, 0], neighbors[:, 1], neighbors[:, 2]
    # return neighbors[occ_map[x, y, z] == False]

    neighbors = index_value + possible_outputs
    neighbors = neighbors[np.all(neighbors >= 0, axis=1), :]
    neighbors = neighbors[np.all(neighbors < graph_shape, axis=1), :]
    # print("neighbors",np.shape(neighbors))
    x, y, z = neighbors[:, 0], neighbors[:, 1], neighbors[:, 2]
    neighbors = neighbors[np.where(occ_map[x, y, z] == False)]
    return neighbors


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

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    # print("START INDEX", start_index)
    # print("GOAL INDEX", goal_index)
    #
    #
    # # Return a tuple (path, nodes_expanded)
    # start_point = start
    # print("start_point", start_point)
    # goal_point = goal
    # print("goal_point", goal_point)
    # map_resolution = resolution
    # print("map resolution", map_resolution)
    # world_object = world
    # print("world_object", world)
    # min_distance = margin
    # print("minimum distance", margin)

    # something = 20
    # full_path = []
    # for i in range(something):
    #     xyz_path = path[i]
    #     full_path = np.append(xyz_path)
    # if path is None:
    #     return None

    # distance_f_dict = default

    # distance_goal_dict = {}
    # distance_cube_dict = {}
    distance_goal_dict = defaultdict(lambda: float("inf"))
    distance_cube_dict = defaultdict(lambda: float("inf"))
    master_dict = {}

    # for i in range(len(occ_map)):
    #     for j in range(len(occ_map[i])):
    #         for k in range(len(occ_map[i][j])):
    #             index = (i, j, k)
    #             distance_goal_dict[index] = float("inf")
    #             distance_cube_dict[index] = float("inf")
    #             master_dict[index] = None

    graph_shape = occ_map.map.shape

    Q_list = [(0, start_index)]
    Y_set = set()

    distance_goal_dict[start_index] = 0
    distance_cube_dict[start_index] = np.linalg.norm((np.array(goal_index) - np.array(start_index)))

    number_nodes = 0

    while len(Q_list) > 0:

        # Remove and return the smallest node from Q_list
        _, closest_node = heappop(Q_list)

        if closest_node == goal_index:
            path = []
            next_node = goal_index
            path.append(goal)

            while next_node is not None:
                parent_node = master_dict[next_node]
                if parent_node == start_index:
                    path.insert(0, start)
                    break
                path.insert(0, occ_map.index_to_metric_center(parent_node))
                next_node = parent_node

            path = np.asarray(path)
            return path, number_nodes

        number_nodes += 1
        Y_set.add(closest_node)

        neighbour_nodes = find_near_cubes(closest_node, graph_shape, occ_map.map)

        # graph_shape = np.array([graph_shape])

        # possible_outputs = np.array([[i, j, k] for i in [-1, 0, 1] for j in [-1, 0, 1] for k in [-1, 0, 1]])
        # # possible_outputs = np.array(
        # #     [[-1, -1, -1], [-1, -1, 0], [-1, -1, 1], [-1, 0, -1], [-1, 0, 0], [-1, 0, 1], [-1, 1, -1],
        # #      [-1, 1, 0], [-1, 1, 1], [0, -1, -1], [0, -1, 0], [0, -1, 1], [0, 0, -1], [0, 0, 1], [0, 1, -1], [0, 1, 0],
        # #      [0, 1, 1], [1, -1, -1], [1, -1, 0], [1, -1, 1], [1, 0, -1], [1, 0, 0], [1, 0, 1], [1, 1, -1], [1, 1, 0],
        # #      [1, 1, 1]])
        # print("shape", np.shape(possible_outputs))
        #
        # neighbours = closest_node + possible_outputs
        # print("nshape", np.shape(neighbours))
        # neighbours = neighbours[np.all(neighbours >= 0, axis=1), :]
        # print("nneighbours", np.shape(neighbours))
        # neighbours = neighbours[np.all(neighbours < graph_shape, axis=1), :]
        # print("neighbours1", np.shape(neighbours))
        # x, y, z = neighbours[:, 0], neighbours[:, 1], neighbours[:, 2]
        # # print('hi')
        # occ_map = occ_map.map
        # neighbour_nodes = neighbours[np.where(occ_map[x, y, z] == False)]
        # print("nodes", neighbour_nodes)

        for neighbor in neighbour_nodes:
            current_neighbor = tuple(neighbor)

            # Continue to the next iteration if current_neighbor is in Y_set
            if current_neighbor in Y_set:
                continue

            # Compute the Euclidean distance between the current_neighbor and the closest_node
            euclidean_distance = np.linalg.norm(np.array(current_neighbor) - np.array(closest_node))

            # Update the distance_cube_dict for the current_neighbor
            cumulative_distance = distance_cube_dict[closest_node] + euclidean_distance

            total_distance = cumulative_distance

            if astar:
                heuristic_distance = np.linalg.norm(np.array(goal_index) - np.array(current_neighbor))
                total_distance = cumulative_distance + heuristic_distance

            if cumulative_distance < distance_cube_dict[current_neighbor]:
                distance_cube_dict[current_neighbor] = cumulative_distance
                distance_goal_dict[current_neighbor] = total_distance
                master_dict[current_neighbor] = closest_node

                # Add the current_neighbor to Q_list
                heappush(Q_list, (total_distance, current_neighbor))

                # print("hi")



    return None, number_nodes
