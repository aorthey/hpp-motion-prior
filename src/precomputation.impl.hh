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

# include <vector>
# include <hpp/model/fwd.hh>
# include <hpp/core/fwd.hh>
# include <hpp/corbaserver/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/config-projector.hh>
# include <hpp/core/deprecated.hh>
# include <hpp/core/problem.hh>
# include <hpp/corbaserver/motion-prior/fwd.hh>

# include "natural-constraints.hh"
# include "precomputation.hh"

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

          /// \brief Precompute the reduced motion planning problem by taking
          //the geometry of the robot into account. The geometry has to be
          //provided in capsule format.
          virtual hpp::floatSeq* robot () throw (hpp::Error);

          /// \brief Add natural constraints to the problemsolver
          virtual hpp::Names_t* addNaturalConstraints
                (const char* prefix, const hpp::floatSeq& dofArray,
                 const char* leftAnkle, const char* rightAnkle) throw (hpp::Error);

        private:
          /// \brief Pointer to the Server owning this object
          corbaServer::Server* server_;

          /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
          /// Instantiated at construction.
          core::ProblemSolverPtr_t problemSolver_;

        };
      } // end of namespace impl.
    } // end of namespace motionprior
  } // end of namespace corbaServer
} // end of namespace hpp.
