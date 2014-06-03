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


#include <algorithm>
#include <vector>
#include "precomputation-utils.hh"

// --------------------------------------------------------------------
// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
// Optimized for HPP framework
namespace hpp
{
  namespace corbaserver
  {
    namespace motionprior
    {
      hpp::floatSeq* vectorToFloatSeq(const vector_t& q){
        hpp::floatSeq* p = new hpp::floatSeq;
        p->length (q.size());
        for(uint i=0;i<q.size();i++){
          (*p)[i] = q[i];
        }
        return p;
      }
      vector_t floatSeqToVector(const hpp::floatSeq &q){
        std::size_t length = (std::size_t)q.length();
        vector_t p; p.resize(length);
        for (std::size_t i = 0; i < length; i++) {
          p[i] = q[i];
        }
        return p;
      }
      hpp::Names_t* stringToNamesT(std::vector<std::string> &str){
        uint size = str.size();
        char** nameList = Names_t::allocbuf(size);
        Names_t *names = new Names_t (size, size, nameList);
        for(std::size_t i = 0; i < str.size (); ++i) {
          std::string it = str.at(i);
          nameList[i] = (char*) malloc (sizeof(char)*(it.length()+1));
          strcpy (nameList[i], it.c_str ());
        }
        return names;
      }
      namespace convexhull
      {
        using namespace std;
        double cross(const ProjectedCapsulePoint &O, const ProjectedCapsulePoint &A, const ProjectedCapsulePoint &B)
        {
          return (A.y - O.y) * (B.z - O.z) - (A.z - O.z) * (B.y - O.y);
        }
         
        // Returns a list of points on the convex hull in counter-clockwise order.
        // Note: the last point in the returned list is the same as the first one.

        std::vector<ProjectedCapsulePoint> convex_hull(std::vector<ProjectedCapsulePoint> P)//no by-reference, we need to sort P!
        {
          uint n = P.size();
          uint k = 0;
          std::vector<ProjectedCapsulePoint> H(2*n);
         
          // Sort points lexicographically
          sort(P.begin(), P.end());
         
          // Build lower hull
          for (int i = 0; i < n; ++i) {
            while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
            H[k++] = P[i];
          }
         
          // Build upper hull
          for (int i = n-2, t = k+1; i >= 0; i--) {
            while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
            H[k++] = P[i];
          }
         
          H.resize(k);
          return H;
        }

      }// end of namespace convexhull
    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
