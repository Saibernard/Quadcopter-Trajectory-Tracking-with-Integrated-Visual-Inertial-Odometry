import numpy as np


from .graph_search import graph_search

# import occupancy_map

# import se3_control


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

        self.resolution = np.array([0.20, 0.20, 0.20])

        self.margin = 0.55


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

        # Initialize the waypoints with the path
        self.waypoints = self.path

        # Calculate the number of waypoints and dimensions
        num_waypoints, dimensions = self.waypoints.shape

        # Initialize the directions array with zeros
        self.directions = np.zeros((num_waypoints - 1, dimensions))

        # Initialize the segment_distances array with zeros
        self.segment_distances = np.zeros((num_waypoints - 1, 1))

        # Initialize the desired_velocities array with zeros
        self.desired_velocities = np.zeros((num_waypoints - 1, dimensions))

        # Initialize the start_times array with zeros
        self.start_times = np.zeros((num_waypoints - 1, 1))

        distance_threshold = 0.31
        max_velocity = 2.4


        self.velocity = 2.2

        time_elapsed = 0

        idx = 0

        while idx < len(self.waypoints) - 1:

            point_diff = self.waypoints[idx + 1] - self.waypoints[idx]

            distance = np.linalg.norm(point_diff)
            print("distance", distance)

            self.segment_distances[idx] = distance

            self.directions[idx] = point_diff / distance

            if distance > distance_threshold:
                self.velocity = max_velocity
            else:
                self.velocity = 2.2

            self.desired_velocities[idx] = self.velocity * self.directions[idx]

            time_elapsed += distance / self.velocity

            self.start_times[idx] = time_elapsed

            idx += 1

        self.desired_velocities = np.append(self.desired_velocities, [np.zeros(3)], axis=0)

        self.directions = np.append(self.directions, [np.zeros(3)], axis=0)

        self.segment_distances = np.append(self.segment_distances, [[0]], axis=0)

        self.directions = np.append(self.directions, [np.zeros(3)], axis=0)

        self.start_times = np.insert(self.start_times, 0, 0)


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

        x, x_dot, x_ddot, x_dddot, x_ddddot = [np.zeros(3) for _ in range(5)]
        yaw = yaw_dot = 0


        # STUDENT CODE HERE

        segment_indices = np.where((self.start_times <= t) & (t < np.append(self.start_times[1:], np.inf)))[0]

        if segment_indices.size > 0:
            i = segment_indices[0]
            x_dot = self.velocity * self.directions[i]
            x = self.waypoints[i] + x_dot * (t - self.start_times[i])
        elif t >= self.start_times[-1]:
            x = self.waypoints[-1]

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,

                            'yaw':yaw, 'yaw_dot':yaw_dot}

        return flat_output
