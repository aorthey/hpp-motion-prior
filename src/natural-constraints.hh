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

#include <hpp/corbaserver/fwd.hh>

namespace hpp
{
  namespace corbaServer
  {
    namespace precomputation
    {
      using namespace hpp::core;
      using namespace hpp::model;
      std::vector <DifferentiableFunctionPtr_t> createNaturalConstraintsManifold
      (const DevicePtr_t& robot, const JointPtr_t& leftAnkle,
       const JointPtr_t& rightAnkle, ConfigurationIn_t configuration);

    } // end of namespace precomputation
  } // end of namespace corbaServer.
} // end of namespace hpp.
