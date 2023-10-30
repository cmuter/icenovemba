# Casper Muter
# ME570 HW 3
import math
import numbers
import numpy as np
from me570_potential import RepulsiveSphere
from me570_potential import SphereWorld


from matplotlib import cm
from matplotlib import pyplot as plt

import os
print("Current working directory:", os.getcwd())

def numel(var):
    """
    Counts the number of entries in a numpy array, or returns 1 for fundamental
    numerical types
    """
    if isinstance(var, numbers.Number):
        size = int(1)
    elif isinstance(var, np.ndarray):
        size = var.size
    else:
        raise NotImplementedError(f'number of elements for type {type(var)}')
    return size


class Grid:
    """
    A function to store the coordinates of points on a 2-D grid and evaluate
    arbitrary functions on those points.
    """
    def __init__(self, xx_ticks, yy_ticks):
        """
        Stores the input arguments in attributes.
        """
        self.xx_ticks = xx_ticks
        self.yy_ticks = yy_ticks

    def eval(self, fun):
        """
        This function evaluates the function  fun (which should be a function)
        on each point defined by the grid.
        """

        dim_domain = [numel(self.xx_ticks), numel(self.yy_ticks)]
        dim_range = [numel(fun(np.array([[0], [0]])))]
        fun_eval = np.nan * np.ones(dim_domain + dim_range)
        for idx_x in range(0, dim_domain[0]):
            for idx_y in range(0, dim_domain[1]):
                x_eval = np.array([[self.xx_ticks[idx_x]],
                                   [self.yy_ticks[idx_y]]])
                fun_eval[idx_x, idx_y, :] = np.reshape(fun(x_eval),
                                                       [1, 1, dim_range[0]])

        # If the last dimension is a singleton, remove it
        if dim_range == [1]:
            fun_eval = np.reshape(fun_eval, dim_domain)
            
        return fun_eval

    def mesh(self):
        """
        Shorhand for calling meshgrid on the points of the grid
        """
        return np.meshgrid(self.xx_ticks, self.yy_ticks)

    def plot_threshold(self, f_handle, threshold=10):
        """
        The function evaluates the function  f_handle on points placed on the grid.
        """
        def f_handle_clip(val):
            return clip(f_handle(val), threshold)

        f_eval = self.eval(f_handle_clip)

        [xx_mesh, yy_mesh] = self.mesh()
        f_dim = numel(f_handle_clip(np.zeros((2, 1))))
        if f_dim == 1:
            # scalar field
            fig = plt.gcf()
            axis = fig.add_subplot(111, projection='3d')

            axis.plot_surface(xx_mesh,
                              yy_mesh,
                              f_eval.transpose(),
                              cmap=cm.gnuplot2)
            axis.set_zlim(0, threshold)
        elif f_dim == 2:
            # vector field

            # grid.eval gives the result transposed with respect to
            # what meshgrid expects
            f_eval = f_eval.transpose((1, 0, 2))
            # vector field
            plt.quiver(xx_mesh,
                       yy_mesh,
                       f_eval[:, :, 0],
                       f_eval[:, :, 1],
                       angles='xy',
                       scale_units='xy',
                       scale=1)
            axis = plt.gca()
        else:
            raise NotImplementedError(
                'Field plotting for dimension greater than two not implemented'
            )

        axis.set_xlim(-15, 15)
        axis.set_ylim(-15, 15)
        plt.xlabel('x')
        plt.ylabel('y')
            
class Sphere:
    """ Class for plotting and computing distances to spheres (circles, in 2-D). """
    def __init__(self, center, radius, distance_influence):
        """
        Save the parameters describing the sphere as internal attributes.
        """
        self.center = center
        self.radius = radius
        self.distance_influence = distance_influence

    def plot(self, color):
        """
        This function draws the sphere (i.e., a circle) of the given radius, and the specified color,
    and then draws another circle in gray with radius equal to the distance of influence.
        """
        # Get current axes
        ax = plt.gca()

        # Add circle as a patch
        if self.radius > 0:
            # Circle is filled in
            kwargs = {'facecolor': (0.3, 0.3, 0.3)}
            radius_influence = self.radius + self.distance_influence
        else:
            # Circle is hollow
            kwargs = {'fill': False}
            radius_influence = -self.radius - self.distance_influence

        center = (self.center[0, 0], self.center[1, 0])
        ax.add_patch(
            plt.Circle(center,
                       radius=abs(self.radius),
                       edgecolor=color,
                       **kwargs))

        ax.add_patch(
            plt.Circle(center,
                       radius=radius_influence,
                       edgecolor=(0.7, 0.7, 0.7),
                       fill=False))

    def distance(self, points):
        """
        Computes the signed distance between points and the sphere, while taking
        into account whether the sphere is hollow or filled in.
        """
        # Compute the Euclidean distance between each point and the center of the sphere
        distances_to_center = np.linalg.norm(points - self.center[:, np.newaxis], axis=0)
        
        # Compute the distance from the points to the surface of the sphere
        d_points_sphere = distances_to_center - abs(self.radius)
        
        # If the sphere is hollow (negative radius), flip the sign of the distances inside the sphere
        if self.radius < 0:
            d_points_sphere[distances_to_center < abs(self.radius)] *= -1

        return d_points_sphere

    def distance_grad(self, points):
        """
        Computes the gradient of the signed distance between points and the
        sphere, consistently with the definition of Sphere.distance.
        """
        # Compute the vector from each point to the center of the sphere
        vectors_to_center = points - self.center[:, np.newaxis]
   
        # Compute the Euclidean distance between each point and the center of the sphere
        distances_to_center = np.linalg.norm(vectors_to_center, axis=0, keepdims=True)
   
        # Handle the special case where a point is at the center of the sphere
        mask_center = distances_to_center == 0
   
        # Compute the gradient of the distance
        grad_d_points_sphere = vectors_to_center / distances_to_center
   
        # Set the gradient to NaN for points at the center of the sphere
        grad_d_points_sphere[:, mask_center[0]] = np.nan

        return grad_d_points_sphere


def clip(val, threshold):
    """
    If val is a scalar, threshold its value; if it is a vector, normalized it
    """
    if isinstance(val, np.ndarray):
        val_norm = np.linalg.norm(val)
        if val_norm > threshold:
            val = val * threshold / val_norm
    elif isinstance(val, numbers.Number):
        if np.isnan(val):
            val = threshold
        else:
            val = min(val, threshold)
    else:
        raise ValueError('Numeric format not recognized')

    return val

def gca_3d():
    """
    Get current Matplotlib axes, and if they do not support 3-D plotting,
    add new axes that support it
    """
    fig = plt.gcf()
    if len(fig.axes) == 0 or not hasattr(plt.gca(), 'plot3D'):
        axis = fig.add_subplot(111, projection='3d')
    else:
        axis = plt.gca()
    return axis



def rot2d(theta):
    """
    Create a 2-D rotation matrix from the angle  theta according to (1).
    """
    rot_theta = np.array([[math.cos(theta), -math.sin(theta)],
                          [math.sin(theta), math.cos(theta)]])
    return rot_theta


def line_linspace(a_line, b_line, t_min, t_max, nb_points):
    """
    Generates a discrete number of  nb_points points along the curve
    (t)=( a(1)t + b(1), a(2)t + b(2))  R^2 for t ranging from  tMin to  tMax.
    """
    t_sequence = np.linspace(t_min, t_max, nb_points)
    theta_points = a_line * t_sequence + b_line
    return theta_points



class Torus:
    """
    A class that holds functions to compute the embedding
    and display a torus and curves on it.
    """

    def phi(self, theta):
        """
        Implements equation (eq:chartTorus).
        """
        nb_points = theta.shape[1]
        x_torus = np.zeros((3, nb_points))

        for idx in range(nb_points):
            angle1 = theta[0, idx].item()
            angle2 = theta[1, idx].item()
            R3 = np.array([[np.cos(angle2), -np.sin(angle2), 0],
                           [np.sin(angle2), np.cos(angle2), 0],
                           [0, 0, 1]])
            xz_crl = np.array([np.cos(angle1), 0, np.sin(angle1)])
            x_torus[:, idx] = R3 @ xz_crl
        return x_torus

    def plot(self):
        # Setting up the grid
        nb_grid_x = nb_grid_y = 33
        theta1 = np.linspace(0, 2*np.pi, nb_grid_x)
        theta2 = np.linspace(0, 2*np.pi, nb_grid_y)
        # Creating an instance of the grid class
        grid_instance = Grid(theta1, theta2)
        # Evaluating function on grid
        torus_coords = grid_instance.eval(self.phi)
        # Extracting xyz coords from torus_coords
        x_coords = torus_coords[:, :, 0]
        y_coords = torus_coords[:, :, 1]
        z_coords = torus_coords[:, :, 2]
        # Plotting surface
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(x_coords, y_coords, z_coords, cmap='viridis')
        ax.set_zlim(-0.75, 0.75)
        plt.show()

    def phi_push_curve(self, a_line, b_line):
        # Generate the points on the curve theta using linspace
        theta_points = line_linspace(a_line, b_line, 0, 1, 31)
        # Evaluating the curve x(t)
        x_points_list = np.array([self.phi(theta.reshape(2, 1)) for theta in theta_points.T])
        x_points = np.concatenate(x_points_list, axis=1)
        return x_points

    def plot_curves(self):
        a_lines = [
        np.array([[3 / 4 * math.pi], [0]]),
        np.array([[3 / 4 * math.pi], [3 / 4 * math.pi]]),
        np.array([[-3 / 4 * math.pi], [3 / 4 * math.pi]]),
        np.array([[0], [-3 / 4 * math.pi]])
        ]

        b_line = np.array([[-1], [-1]])

        axis = gca_3d()
        self.plot()  # Ensure the torus is plotted
        for a_line in a_lines:
            x_points = self.phi_push_curve(a_line, b_line)
            axis.plot(x_points[0, :], x_points[1, :], x_points[2, :], linewidth=2)

    def phi_test(self):
        """
        Uses the function phi to plot two perpendicular rings
        """
        nb_points = 200
        theta_ring = np.linspace(0, 15 / 8 * np.pi, nb_points)
        theta_zeros = np.zeros((1, nb_points))
        data = [
            np.vstack((theta_ring, theta_zeros)),
            np.vstack((theta_zeros, theta_ring))
        ]
        axis = gca_3d()
        for theta in data:
            ring = np.zeros((3, nb_points))
            for idx in range(nb_points):
                ring[:, [idx]] = self.phi(theta[:, [idx]])
            axis.plot(ring[0, :], ring[1, :], ring[2, :])


class Polygon:
    # Constructor for the polygon class
    def __init__(self, vertices):
        self.vertices = vertices

    # Method to reverse the order of vertices
    def flip(self):
        self.vertices = np.flip(self.vertices, axis=1)

    # Method to plot the polygon
    def plot(self, style, alpha=1):
        print("Plotting polygon")
        # Extracting the x and y coordinates of the vertices
        x = self.vertices[0, :]
        y = self.vertices[1, :]
        # Closing the polygon by adding the first vertex
        x = np.append(x, x[0])
        y = np.append(y, y[0])
        if self.is_filled():
            plt.fill(x, y, style, alpha=alpha)  # Fill the inside of the polygon
        # Plotting the polygon using arrows to indicate order of vertices
        plt.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1],
                   angles='xy', scale_units='xy', scale=1, color=style)

    def is_filled(self):
        # Checks the ordering of the vertices
        area = 0
        n = len(self.vertices[0])
        for i in range(n):
            x1, y1 = self.vertices[0][i], self.vertices[1][i]
            x2, y2 = self.vertices[0][(i + 1) % n], self.vertices[1][(i + 1) % n]
            area += (x1 * y2) - (x2 * y1)
        return area > 0

    def is_self_occluded(self, idx_vertex, point):
        # Gets the vertex, vertex_prev, and vertex_next
        vertex = self.vertices[:, idx_vertex:idx_vertex+1]
        vertex_prev = self.vertices[:, (idx_vertex - 1):(idx_vertex - 1) + 1]
        vertex_next = self.vertices[:, (idx_vertex + 1):(idx_vertex + 1) + 1]
        # Check if vertex_prev/next coincide with vertex
        if np.array_equal(vertex, vertex_prev) or np.array_equal(vertex, vertex_next):
            return False
        # Check if inside cone
        edge_angle = angle(vertex, vertex_prev, vertex_next)
        point_angle = angle(vertex, point, vertex_next)
        if point_angle <= edge_angle:
            flag_point = False
        else:
            flag_point = True
        return flag_point

    def is_visible(self, idx_vertex, test_points):
        # Initialize visibility flags to True for all test points
        visibility_flags = [True] * test_points.shape[1]
        # Get the coordinates of the given vertex
        vertex_coords = self.vertices[:, idx_vertex]
        # Iterate over each test point
        for i, point in enumerate(test_points.T):
            # Check if the test point is a vertex of the polygon
            if np.any(np.all(self.vertices == point[:, np.newaxis], axis=0)):
                # The test point is a vertex of the polygon
                if self.is_filled():
                    visibility_flags[i] = False
                else:
                    visibility_flags[i] = True
                    continue
            # Check for self occlusion
            if self.is_self_occluded(idx_vertex, point.reshape(2, 1)):
                visibility_flags[i] = False
                continue
            # Create an edge from the vertex to the test point
            test_edge = Edge(np.column_stack((vertex_coords, point)))
            # Check for intersections with each edge of the polygon
            for j in range(len(self.vertices[0])):
                polygon_edge_coords = self.vertices[:, [j, (j+1) % len(self.vertices[0])]]
                polygon_edge = Edge(polygon_edge_coords)
                # If there's an intersection, set visibility to False
                if test_edge.is_collision(polygon_edge):
                    visibility_flags[i] = False
                    break
        return visibility_flags

    def is_collision(self, test_points):
        collision_flags = np.zeros(test_points.shape[1], dtype=bool)
        filled = self.is_filled()
        for i, point in enumerate(test_points.T):
            x, y = point
            odd_nodes = False
            j = len(self.vertices[0]) - 1
            for k in range(len(self.vertices[0])):
                xi, yi = self.vertices[0][k], self.vertices[1][k]
                xj, yj = self.vertices[0][j], self.vertices[1][j]
                if yi < y and yj >= y or yj < y and yi >= y:
                    if xi + (y - yi) / (yj - yi) * (xj - xi) < x:
                        odd_nodes = not odd_nodes
                j = k
            inside = odd_nodes
            if filled:
                collision_flags[i] = inside
            else:
                collision_flags[i] = not inside
        return collision_flags


class Edge:
    def __init__(self, vertices):
        self.vertices = vertices

    def is_collision(self, edge):
        # Initialize the flag as False (assuming no intersection)
        flag = False
        # Extract the endpoints of the current edge
        x1, y1 = self.vertices[:, 0]
        x2, y2 = self.vertices[:, 1]
        # Extract the endpoints of the other edge
        x3, y3 = edge.vertices[:, 0]
        x4, y4 = edge.vertices[:, 1]
        # Compute the determinants
        det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        # If the determinant is zero, the lines are parallel
        if det == 0:
            return flag
        # Otherwise, compute the intersection point
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2)
              * (x3 * y4 - y3 * x4)) / det
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2)
              * (x3 * y4 - y3 * x4)) / det
        # Check if the intersection point lies within both line segments
        if (min(x1, x2) <= px <= max(x1, x2) and
            min(y1, y2) <= py <= max(y1, y2) and
            min(x3, x4) <= px <= max(x3, x4) and
            min(y3, y4) <= py <= max(y3, y4)):
            flag = True
        return flag

    def plot(self, *args, **kwargs):
        plt.plot(self.vertices[0, :], self.vertices[1, :], *args, **kwargs)


def angle(vertex0, vertex1, vertex2, angle_type='unsigned'):
    """
    Compute the angle between two edges  vertex0-- vertex1 and  vertex0--
    vertex2 having an endpoint in common. The angle is computed by starting
    from the edge  vertex0-- vertex1, and then ``walking'' in a
    counterclockwise manner until the edge  vertex0-- vertex2 is found.
    """
    # Tolerance to check for coincident points
    tol = 2.22e-16
    # Compute vectors corresponding to the two edges, and normalize
    vec1 = vertex1 - vertex0
    vec2 = vertex2 - vertex0
    norm_vec1 = np.linalg.norm(vec1)
    norm_vec2 = np.linalg.norm(vec2)
    if norm_vec1 < tol or norm_vec2 < tol:
        # Vertex1 or vertex2 coincides with vertex0, abort
        edge_angle = math.nan
        return edge_angle
    vec1 = vec1 / norm_vec1
    vec2 = vec2 / norm_vec2
    # Transform vec1 and vec2 into flat 3-D vectors,
    # so that they can be used with np.inner and np.cross
    vec1flat = np.vstack([vec1, 0]).flatten()
    vec2flat = np.vstack([vec2, 0]).flatten()
    c_angle = np.inner(vec1flat, vec2flat)
    s_angle = np.inner(np.array([0, 0, 1]), np.cross(vec1flat, vec2flat))
    edge_angle = math.atan2(s_angle, c_angle)
    angle_type = angle_type.lower()
    if angle_type == 'signed':
        pass
    elif angle_type == 'unsigned':
        edge_angle = (edge_angle + 2 * math.pi) % (2 * math.pi)
    else:
        raise ValueError('Invalid argument angle_type')
    return edge_angle


if __name__ == "__main__":
    # Load the sphere world
    sphere_world_instance = SphereWorld()
    
    # Ensure that each sphere in the world has the correct influence distance set
    for sphere in sphere_world_instance.world:
        sphere.distance_influence = 10  # Set the influence distance to 10 units as per the PDF
    
    # Define the ticks for the grid
    xx_ticks = np.linspace(-15, 15, 100)  # 100 points from -15 to 15 for x-axis
    yy_ticks = np.linspace(-15, 15, 100)  # 100 points from -15 to 15 for y-axis
    grid_instance = Grid(xx_ticks, yy_ticks)
    
    # For the first two spheres in the sphere world
    for i in range(2):
        # Create a RepulsiveSphere object for the current sphere
        rep_sphere = RepulsiveSphere(sphere_world_instance.world[i])
        
        # Check if the sphere has the distance_influence attribute set
        if not hasattr(rep_sphere.sphere, "distance_influence"):
            raise ValueError(f"Sphere {i+1} does not have the distance_influence attribute set.")
        
        # Use the plot_threshold function to visualize the gradient of the repulsive potential
        plt.figure()  # Create a new figure for each sphere
        grid_instance.plot_threshold(rep_sphere.grad)
        
        # Overlap the plot of the sphere
        sphere_world_instance.world[i].plot('b')  # Plot the sphere in blue for clarity
        
        # Set title and show the plot
        plt.title(f"Gradient of Repulsive Potential for Sphere {i+1}")
        plt.show()
    

    """
    square = np.array([[0, 0, 1, 1], [0, 1, 1, 0]])
    polygon = Polygon(square)
    # Plotting the filled-in polygon
    plt.figure(figsize=(6, 6))  # Create a new figure
    polygon.plot(style='r')
    plt.title("Hollow Polygon")
    plt.show()  # Display Plot
    # Flipping the polygon to make it hollow
    polygon.flip()
    # Plotting the hollow polygon
    plt.figure(figsize=(6, 6))  # Create a new figure
    polygon.plot(style='b')
    plt.title("Filled in Polygon")
    plt.show()  # Display Plot
    torus_instance = Torus()
    torus_instance.plot()
    torus_instance.phi_test()
    torus_instance.plot_curves()
    plt.show()
    """