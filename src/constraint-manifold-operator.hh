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


namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {
      class ConstraintManifoldOperator{

      public:

        ConstraintManifoldOperator(hpp::core::ProblemSolverPtr_t &problemSolver);

        void init() throw (hpp::Error);
        
        double apply( Configuration_t &q ) throw (hpp::Error);

        void reset() throw (hpp::Error);

        bool success();

      private:

        bool success_;

        core::ProblemSolverPtr_t problemSolver_;

        std::vector <DifferentiableFunctionPtr_t> constraintSet;

      };
    } // end of namespace motionprior.
  } // end of namespace corbaServer.
} // end of namespace hpp.

