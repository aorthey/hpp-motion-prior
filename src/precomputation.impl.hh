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

# include <hpp/corbaserver/motion-prior/fwd.hh>

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

          /// \brief get volume of the surface area, inscribed in the convex hull
          /// of the projected capsule points
          double getVolume () throw (hpp::Error);

          hpp::floatSeq* getRandomIrreducibleConfiguration () throw (hpp::Error);
          hpp::floatSeq* getRandomConfiguration () throw (hpp::Error);

        private:
          virtual hpp::floatSeq* shootRandomConfig() throw (hpp::Error);

          virtual vector_t shootRandomConfigVector() throw (hpp::Error);

          /// \brief Compute q = q + lambda*q', i.e. one update step of gradient
          // descent
          Configuration_t step (const Configuration_t &q, const Configuration_t &qq, double lambda) throw (hpp::Error);

          /// \brief compute the gradient wrt to the outer convex hull points and
          /// its associated jacobians
          virtual Configuration_t getGradientFromConfiguration(const Configuration_t &q) throw (hpp::Error);

          void computeProjectedConvexHullFromCurrentConfiguration() throw (hpp::Error);

          void computeProjectedConvexHullFromConfiguration(Configuration_t &q) throw (hpp::Error);

          ProjectedCapsulePointVectorPtr_t getProjectedConvexHullFromConfiguration (const Configuration_t &q)
          throw (hpp::Error);

          bool projectOntoConstraintManifold (Configuration_t &q) throw (hpp::Error);
          bool projectOntoIrreducibleManifold(Configuration_t &q) throw (hpp::Error);
          bool projectOntoConstraintIrreducibleManifold (Configuration_t &q) throw (hpp::Error);

        private:

          /// \brief Pointer to the Server owning this object
          corbaServer::Server* server_;

          /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
          /// Instantiated at construction.
          core::ProblemSolverPtr_t problemSolver_;

          /// Internal State
          ProjectedCapsulePointVectorPtr_t cvxCaps_;
          Configuration_t q_;
          
          ConstraintManifoldOperatorPtr_t cnstrOp_;
        };
      } // end of namespace impl.
    } // end of namespace motionprior
  } // end of namespace corbaServer
} // end of namespace hpp.
