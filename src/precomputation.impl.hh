// Copyright (c) 2014 CNRS
// Author: Andreas Orthey
//
// This file is part of the hpp-motion-prior.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#pragma once

# include <iostream>
# include <cmath>
# include <vector>

# include <hpp/core/config.hh>
# include <hpp/core/config-projector.hh>
# include <hpp/core/deprecated.hh>
# include <hpp/core/problem.hh>
# include <hpp/corbaserver/motion-prior/fwd.hh>

# include "natural-constraints.hh"
# include "precomputation.hh"
# include "capsule-parser.hh"

namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {

      namespace impl
      {

        /// \brief Implement CORBA interface ``Precomputation''.
        class Precomputation : public virtual POA_hpp::corbaserver::motion_prior::Precomputation
        {
        public:

          //Precomputation (corbaServer::Server* server);
          Precomputation ();
	  void setProblemSolver (const ProblemSolverPtr_t& problemSolver);

          /// \brief returns the points of the convex hull of the projected
          /// capsules
          virtual hpp::floatSeq* getConvexHullCapsules () throw (hpp::Error);

          /// \brief compute the gradient wrt to the outer convex hull points and
          /// its associated jacobians
          virtual hpp::floatSeq* getGradient () throw (hpp::Error);

          /// \brief get volume of the surface area, inscribed in the convex hull
          /// of the projected capsule points
          virtual double getVolume () throw (hpp::Error);

          /// \brief Use the current configuration and project it down until it is
          ///  irreducible up to a threshold
          virtual hpp::floatSeq* projectUntilIrreducible () throw (hpp::Error);

          /// \brief Perform one step of the gradient descent projection onto the
          ///  irreducible manifold
          virtual hpp::floatSeq* projectUntilIrreducibleOneStep () throw (hpp::Error);

          virtual void setCurrentConfiguration (const hpp::floatSeq& dofArray) throw (hpp::Error);
          virtual void setCurrentConfiguration (const vector_t& q) throw (hpp::Error);

          //---------------------------------------------------------------------
          // NATURAL CONSTRAINTS
          //---------------------------------------------------------------------
          virtual hpp::Names_t* addNaturalConstraints
                (const char* prefix, const hpp::floatSeq& dofArray,
                 const char* leftAnkle, const char* rightAnkle) throw (hpp::Error);

        private:
          /// \brief Compute q = q + lambda*q', i.e. one update step of gradient
          // descent
          virtual vector_t updateConfiguration(const vector_t &qq, double lambda) throw (hpp::Error);

          virtual vector_t getGradientVector();

          void computeProjectedConvexHullFromCurrentConfiguration() throw (hpp::Error);

        private:
          /// \brief Pointer to the Server owning this object
          corbaServer::Server* server_;

          /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
          /// Instantiated at construction.
          core::ProblemSolverPtr_t problemSolver_;

          std::vector<ProjectedCapsulePoint> cvxCaps_;
        };
      } // end of namespace impl.
    } // end of namespace motionprior
  } // end of namespace corbaServer
} // end of namespace hpp.
