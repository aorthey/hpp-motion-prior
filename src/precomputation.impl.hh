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

          virtual void setCurrentConfiguration (const hpp::floatSeq& dofArray) throw (hpp::Error);
          virtual void setCurrentConfiguration (const Configuration_t& q) throw (hpp::Error);

          virtual hpp::floatSeq* shootRandomConfig() throw (hpp::Error);
          virtual vector_t shootRandomConfigVector() throw (hpp::Error);

          hpp::floatSeq* getRandomIrreducibleConfiguration () throw (hpp::Error);

        private:

          /// \brief compute the gradient wrt to the outer convex hull points and
          /// its associated jacobians
          virtual hpp::floatSeq* getGradient () throw (hpp::Error);
          /// \brief Compute q = q + lambda*q', i.e. one update step of gradient
          // descent
          virtual Configuration_t step(const Configuration_t &qq, double lambda) throw (hpp::Error);

          virtual Configuration_t getGradientVector() throw (hpp::Error);
          virtual Configuration_t getGradientVector(const Configuration_t &q) throw (hpp::Error);

          void computeProjectedConvexHullFromCurrentConfiguration() throw (hpp::Error);

          void computeProjectedConvexHullFromConfiguration(vector_t &q) throw (hpp::Error);

          ProjectedCapsulePointVectorPtr_t getProjectedConvexHullFromConfiguration (vector_t &q);
          bool projectOntoConstraintManifold(vector_t &q) throw (hpp::Error);
          bool projectOntoIrreducibleManifold(vector_t &q) throw (hpp::Error);

        private:

          /// \brief Pointer to the Server owning this object
          corbaServer::Server* server_;

          /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
          /// Instantiated at construction.
          core::ProblemSolverPtr_t problemSolver_;

          ProjectedCapsulePointVectorPtr_t cvxCaps_;
          
          ConstraintManifoldOperatorPtr_t cnstrOp_;
        };
      } // end of namespace impl.
    } // end of namespace motionprior
  } // end of namespace corbaServer
} // end of namespace hpp.
