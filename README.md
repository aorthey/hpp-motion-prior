hpp-motion-prior
================

Precomputation module for the hpp-framework

Conceptually idea of this module: Implementation of routines which precompute
information about a given mechanical system or about a given set of environments. Given this
information, the goal is to lower the complexity of the motion planning problem.

Ideally, one would provide an environment, a mechanical system, and the module
provides a simplified motion planning problem, which can be solved faster.
