"""
Classes to define potential and potential planner for the sphere world
"""
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy import io as scio

import me570_geometry


class SphereWorld:
    """ Class for loading and plotting a 2-D sphereworld. """
    def __init__(self):
        """
        Load the sphere world from the provided file sphereworld.mat, and sets the
    following attributes:
     -  world: a  nb_spheres list of  Sphere objects defining all the spherical obstacles in the
    sphere world.
     -  x_start, a [2 x nb_start] array of initial starting locations (one for each column).
     -  x_goal, a [2 x nb_goal] vector containing the coordinates of different goal locations (one
    for each column).
        """
        data = scio.loadmat('sphereworld.mat')

        self.world = []
        for sphere_args in np.reshape(data['world'], (-1, )):
            sphere_args[1] = sphere_args[1].item()
            sphere_args[2] = sphere_args[2].item()
            self.world.append(me570_geometry.Sphere(*sphere_args))

        self.x_goal = data['xGoal']
        self.x_start = data['xStart']
        self.theta_start = data['thetaStart']

    def plot(self, axes=None):
        """
        Uses Sphere.plot to draw the spherical obstacles together with a  * marker at the goal
        location.
        """

        if axes is None:
            axes = plt.gca()

        for sphere in self.world:
            sphere.plot('r')

        plt.scatter(self.x_goal[0, :], self.x_goal[1, :], c='g', marker='*')

        plt.xlim([-11, 11])
        plt.ylim([-11, 11])
        plt.axis('equal')


class RepulsiveSphere:
    """ Repulsive potential for a sphere """
    def __init__(self, sphere):
        """
        Save the arguments to internal attributes
        """
        self.sphere = sphere

    def eval(self, x_eval):
        """s
        Evaluate the repulsive potential from  sphere at the location x= x_eval. The function returns
    the repulsive potential as given by      (  eq:repulsive  ).
        """
        distance = self.sphere.distance(x_eval)

        distance_influence = self.sphere.distance_influence
        if (distance > distance_influence):
            u_rep = 0
        elif (distance_influence > distance > 0):
            u_rep = ((distance**-1 - distance_influence**-1)**2) / 2.
            u_rep = u_rep.item()
        else:
            u_rep = math.nan
        return u_rep

    def grad(self, x_eval):  
        """
        Compute the gradient of U_rep for a single sphere.
        """
        # Compute the gradient of the distance and the distance to the sphere
        grad_distance = self.sphere.distance_grad(x_eval)
        distance_to_sphere = self.sphere.distance(x_eval)
    
        # Define the influence distance
        d_influence = self.sphere.distance_influence
    
        # Compute the gradient for points satisfying the condition
        condition = (0 < distance_to_sphere) & (distance_to_sphere < d_influence)
        grad_u_rep = np.where(
            condition[:, np.newaxis],
            - (1/distance_to_sphere - 1/d_influence) * (1/(distance_to_sphere**2)) * grad_distance,
            0
            )
    
        # Handle the case where the potential is undefined
        undefined_condition = ~condition & (distance_to_sphere <= 0)
        grad_u_rep[:,undefined_condition] = np.nan

        return grad_u_rep


class Attractive:
    """ Repulsive potential for a sphere """
    def __init__(self, potential):
        """
        Save the arguments to internal attributes
        """
        self.potential = potential

    def eval(self, x_eval):
        """
        Evaluate the attractive potential  U_ attr at a point  xEval with respect to a goal location
    potential.xGoal given by the formula: If  potential.shape is equal to  'conic', use p=1. If
    potential.shape is equal to  'quadratic', use p=2.
        """
        x_goal = self.potential['x_goal']
        shape = self.potential['shape']
        if (shape == 'conic'):
            expo = 1
        else:
            expo = 2
        u_attr = np.linalg.norm(x_eval - x_goal)**expo
        return u_attr

    def grad(self, x_eval):
        """
        Evaluate the gradient of the attractive potential  U_ attr at a point  xEval. The gradient
        is given by the formula If  potential['shape'] is equal to 'conic', use p=1; if it is
        equal to 'quadratic', use p=2.
        """
        pass  # Substitute with your code
        return grad_u_attr


class Total:
    """ Combines attractive and repulsive potentials """
    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes
        """
        self.world = world
        self.potential = potential

    def eval(self, x_eval):
        """
        Compute the function U=U_attr+a*iU_rep,i, where a is given by the variable
    potential.repulsiveWeight
        """
        pass  # Substitute with your code
        return u_eval

    def grad(self, x_eval):
        """
        Compute the gradient of the total potential,  U=U_ attr+a*U_rep,i, where a is given by
        the variable  potential.repulsiveWeight
        """
        pass  # Substitute with your code
        return grad_u_eval


class Planner:
    """
    A class implementing a generic potential planner and plot the results.
    """
    def __init__(self):
        """
        Save the arguments to internal attributes
        """
        pass  # Substitute with your code

    def run(self, x_start):
        """
        This function uses a given function (given by  control) to implement a
        generic potential-based planner with step size  epsilon, and evaluates
        the cost along the returned path. The planner must stop when either the
        number of steps given by  nb_stepsis reached, or when the norm of the
        vector given by  control is less than 5 10^-3 (equivalently,  5e-3).
        """
        pass  # Substitute with your code
        return x_path, u_path


class Clfcbf_Control:
    """
    A class implementing a CLF-CBF-based control framework.
    """
    def __init__(self, world, potential):
        """
        Save the arguments to internal attributes, and create an attribute
        attractive with an object of class  Attractive using the argument
        potential.
        """
        self.world = world
        self.potential = potential
        self.attractive = Attractive(potential)

    def function(self, x_eval):
        """
        Evaluate the CLF (i.e.,  self.attractive.eval()) at the given input.
        """
        return self.attractive.eval(x_eval)

    def control(self, x_eval):
        """
        Compute u^* according to      (  eq:clfcbf-qp  ).
        """
        pass  # Substitute with your code
        return u_opt
