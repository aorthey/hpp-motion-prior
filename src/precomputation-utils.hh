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

# include "precomputation.hh"
# include "precomputation.impl.hh"

namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {

      namespace capsules{
        struct ProjectedCapsulePoint;
      }

      vector_t floatSeqToVector(const hpp::floatSeq &q);

      hpp::floatSeq* vectorToFloatSeq(const vector_t& q);

      hpp::Names_t* stringToNamesT(std::vector<std::string> &str);

      namespace convexhull
      {
        /// \brief Implementation of Andrew's monotone chain 2D convex hull algorithm.
        /// Asymptotic complexity: O(n log n).
        /// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
        /// Original source code from
        /// http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C.2B.2B
        bool lexicographicallyCapsuleSort (const ProjectedCapsulePointPtr_t lhs, const ProjectedCapsulePointPtr_t rhs);

        double cross(const ProjectedCapsulePointPtr_t O, const ProjectedCapsulePointPtr_t A, const ProjectedCapsulePointPtr_t B);
         
        // \brief Returns a list of points on the convex hull in counter-clockwise order.
        // Note: the last point in the returned list is the same as the first one.
        ProjectedCapsulePointVectorPtr_t convex_hull(ProjectedCapsulePointVectorPtr_t P);

      }// end of namespace convexhull
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
