#hpp-motion-prior
================

Precomputation module for the hpp-framework

Conceptually idea of this module: Implementation of routines which precompute
information about a given mechanical system or about a given set of environments. Given this
information, the goal is to lower the complexity of the motion planning problem.

Ideally, one would provide an environment, a mechanical system, and the module
provides a simplified motion planning problem, which can be solved faster.
#Python Irreducible Scripts
#### Depends
 * python-scipy
 * python-numpy
 * matplotlib

sudo apt-get install python-numpy python-scipy

##Irreducible Motion Planning for Linear Linkages

We have implemented the curvature projection algorithm from the T-RO paper as a python module called IrreducibleProjector. 

An example for the left arm of the humanoid robot HRP-2. We have a linear linkage structure L0->L1->L2->L3, with l0=0.25m and delta_0 = 0.08m. Let X be the discrete set of locations of L0 over time. Then we can compute the irreducible configurations in a python script as

    L = np.array((0.25,0.25,0.25))
    D = np.array((0.08,0.08,0.08,0.08))
    P = IrreducibleProjector(X,L,D)
    
To compute a specific configuration at a specific time t0 do

    P.getJointAnglesAtT(t0)
    
To visualize the configuration at t0

    P.plotLinearLinkageAtT(t0)
    
To compute the configurations over the complete trajectory of L0 and display each configuration for 0.0001s do

    P.visualizeLinearLinkageProjection(0.0001)
    
## Irreducible Motion Planning from Data Points (quick-and-dirty)

Given a set of data points [X_1,...,X_M] for the root link L_0, the maximum curvature kappa, and the radius of L_0, delta_0, we can compute the maximum number N of sublinks, and its configuration for each discrete root link position (by spline interpolating the shape of the underlying function, so make sure that the points are dense).  This has been implemented in the following script: scripts/irreducible-sublink-projector.py

You can provide a txt file (see example in data-traj/spheretraj.txt), and obtain the sublink configurations for each discrete data point in another txt file. From there, you can plot the sublinks, or use them inside of a controller.
