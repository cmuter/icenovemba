"""
 Please merge the functions and classes from this file with the same file from the previous
 homework assignment
"""
import matplotlib.pyplot as plt
import numpy as np
import me570_geometry as geometry
import me570_potential as pot


class TwoLink:
    """ See description from previous homework assignments. """
    def jacobian_matrix(self, theta):
        """
        Compute the matrix representation of the Jacobian of the position of the end effector with
    respect to the joint angles as derived in Question~ q:jacobian-matrix.
        """
        theta_1 = theta[0]
        theta_2 = theta[1]

        # Defining the length of the links
        l_1 = 5  # Assuming the length of the first link is 5 units. Adjust if needed.
        l_2 = 5  # Assuming the length of the second link is 5 units. Adjust if needed.

        # Define the Jacobian matrix based on the provided information and the kinematics of the two-link manipulator
        j_theta = np.array([
            [-l_1 * np.sin(theta_1) - l_2 * np.sin(theta_1 + theta_2), -l_2 * np.sin(theta_1 + theta_2)],
            [l_1 * np.cos(theta_1) + l_2 * np.cos(theta_1 + theta_2), l_2 * np.cos(theta_1 + theta_2)]
        ])

        return j_theta

    def animate(self, theta):
        """
        Draw the two-link manipulator for each column in theta with a small pause between each drawing operation
        """
        theta_steps = theta.shape[1]
        for i_theta in range(0, theta_steps, 15):
            self.plot(theta[:, [i_theta]], 'k')


class TwoLinkPotential:
    """ Combines attractive and repulsive potentials """
    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes
        """
        pass  # Substitute with your code

    def eval(self, theta_eval):
        """
        Compute the potential U pulled back through the kinematic map of the two-link manipulator,
        i.e., U(Wp_eff(theta)), where U is defined as in Question~q:total-potential, and
        Wp_ eff(theta) is the position of the end effector in the world frame as a function of the joint angles   = _1\\ _2.
        """
        # Assuming you have a method or function that computes the potential U for a given position
        # For this example, I'm using a placeholder function U(position)
        # You should replace this with the actual function or method from Question code 2.3

        two_link = TwoLink()
        position_effector = two_link.kinematic_map(theta_eval)[0]  # Getting the position of the end effector
        u_eval_theta = U(position_effector)  # Placeholder function, replace with actual implementation

        return u_eval_theta

    def grad(self, theta_eval):
        """
        Compute the gradient of the potential U pulled back through the kinematic map of the
        two-link manipulator, i.e., grad U(  Wp_ eff(  )).
        """
        pass  # Substitute with your code
        return grad_u_eval_theta

    def run_plot(self, epsilon, nb_steps):
        """
        This function performs the same steps as Planner.run_plot in
        Question~q:potentialPlannerTest, except for the following:
     - In step  it:grad-handle:  planner_parameters['U'] should be set to  @twolink_total, and
    planner_parameters['control'] to the negative of  @twolink_totalGrad.
     - In step  it:grad-handle: Use the contents of the variable  thetaStart instead of  xStart to
    initialize the planner, and use only the second goal  x_goal[:,1].
     - In step  it:plot-plan: Use Twolink.plotAnimate to plot a decimated version of the results of
    the planner. Note that the output  xPath from Potential.planner will really contain a sequence
    of join angles, rather than a sequence of 2-D points. Plot only every 5th or 10th column of
    xPath (e.g., use  xPath(:,1:5:end)). To avoid clutter, plot a different figure for each start.
        """
        sphere_world = pot.SphereWorld()

        nb_starts = sphere_world.theta_start.shape[1]

        planner = pot.Planner(function=self.eval,
                              control=self.grad,
                              epsilon=epsilon,
                              nb_steps=nb_steps)

        two_link = TwoLink()

        for start in range(0, nb_starts):
            # Run the planner
            theta_start = sphere_world.theta_start[:, [start]]
            theta_path, u_path = planner.run(theta_start)

            # Plots
            _, axes = plt.subplots(ncols=2)
            axes[0].set_aspect('equal', adjustable='box')
            plt.sca(axes[0])
            sphere_world.plot()
            two_link.animate(theta_path)
            axes[1].plot(u_path.T)

class TwoLink:
    def kinematic_map(self, theta):
        theta_1 = theta[0]
        theta_2 = theta[1]

        # Defining the position of the end effector
        end_effector_pos_b2 = np.array([[5], [0]])

        # Define rigid body transformation matrices
        rotation_w_b1 = geometry.rot2d(theta_1)
        rotation_b1_b2 = geometry.rot2d(theta_2)
        rotation_w_b2 = np.matmul(rotation_w_b1, rotation_b1_b2)
        translation_b1_b2 = np.array([[5], [0]])

        # Create links of the manipulator
        polygon1_transf, polygon2_transf = polygons_generate()

        # Transform polygons
        polygon1_transf.vertices = np.matmul(rotation_w_b1, polygon1_transf.vertices)
        polygon2_transf.vertices = np.matmul(rotation_w_b2, polygon2_transf.vertices) + np.matmul(rotation_w_b1, translation_b1_b2)

        vertex_effector_transf = np.matmul(rotation_w_b2, end_effector_pos_b2) + np.matmul(rotation_w_b1, translation_b1_b2)

        return vertex_effector_transf, polygon1_transf, polygon2_transf

    def plot(self, theta, color, alpha=1.0):
        """
        This function should use TwoLink.kinematic_map
        from the previous question together with
        the method Polygon.plot from Homework 1 to plot the manipulator.
        """
        [vertex_effector_transf, polygon1_transf,
         polygon2_transf] = self.kinematic_map(theta)
        polygon1_transf.plot(color, alpha=alpha)
        polygon2_transf.plot(color, alpha=alpha)

    def is_collision(self, theta, points):
        """
        For each specified configuration, returns  True if
        any of the links of the manipulator
        collides with  any of the points, and  False otherwise.
        Use the function
        Polygon.is_collision to check if each link of the
        manipulator is in collision.
        """
        # Initialize an empty list
        collision_flags = []

        # Iterate over each configuration
        for theta_i in theta.T:
            # Get the transformed polygons for the current configuration
            _, polygon1_transf, polygon2_transf = self.kinematic_map(theta_i)

            # Check for collisions with the provided points for each link
            collision1 = any(polygon1_transf.is_collision(points))
            collision2 = any(polygon2_transf.is_collision(points))
            collision_flags.append(collision1 or collision2)

        # Convert the list of collision flags to a numpy array and return
        flag_theta = np.array(collision_flags)
        return flag_theta

    def plot_collision(self, theta, points):
        """
        This function should:
     - Use TwoLink.is_collision for determining if each
     configuration is a collision or not.
     - Use TwoLink.plot to plot the manipulator for all
     configurations, using a red color when the
    manipulator is in collision, and green otherwise.
     - Plot the points specified by  points as black asterisks.
        """
        NTheta = theta.shape[1]
        collision_flags = self.is_collision(theta, points)

        for i in range(NTheta):
            theta_i = theta[:, i]
            if collision_flags[i]:
                self.plot(theta_i, 'r')
            else:
                self.plot(theta_i, 'g')

        # Plot the points as black asterisks
        plt.scatter(points[0, :], points[1, :], marker='*', color='black')
        plt.title("Manipulator Configurations and Collision Points")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.axis('equal')  # Ensure equal scaling for both axes
        plt.show()

    def jacobian(self, theta, theta_dot):
        """
        Implement the map for the Jacobian of the position of
        the end effector with respect to the
        joint angles as derived in Question~ q:jacobian-effector.
        """
        assert theta.ndim == 2, "theta must be a 2D array"
        nb_theta = theta.shape[1]
        vertex_effector_dot = np.zeros((2, nb_theta))
        # Defining the length of the links
        l_1 = 5
        l_2 = 5

        for i in range(nb_theta):
            theta_1, theta_2 = theta[:, i]

            # Define the Jacobian matrix as per the expression above
            J = np.array([
                [-l_1*np.sin(theta_1) - l_2*np.sin(theta_1 + theta_2), -l_2*np.sin(theta_1 + theta_2)],
                [l_1*np.cos(theta_1) + l_2*np.cos(theta_1 + theta_2), l_2*np.cos(theta_1 + theta_2)]
                ])

            vertex_effector_dot[:, i] = np.dot(J, theta_dot[:, i])
        return vertex_effector_dot


def polygons_add_x_reflection(vertices):
    """
    Given a sequence of vertices, adds other vertices by reflection
    along the x axis
    """
    vertices = np.hstack([vertices, np.fliplr(np.diag([1, -1]).dot(vertices))])
    return vertices


def polygons_generate():
    """
    Generate the polygons to be used for the two-link manipulator
    """
    vertices1 = np.array([[0, 5], [-1.11, -0.511]])
    vertices1 = polygons_add_x_reflection(vertices1)
    vertices2 = np.array([[0, 3.97, 4.17, 5.38, 5.61, 4.5],
                          [-0.47, -0.5, -0.75, -0.97, -0.5, -0.313]])
    vertices2 = polygons_add_x_reflection(vertices2)
    return (geometry.Polygon(vertices1), geometry.Polygon(vertices2))


polygons = polygons_generate()
