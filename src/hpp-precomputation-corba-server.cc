// Copyright (C) 2014 LAAS-CNRS
// Author: Andreas Orthey
//
// This file is part of the hpp-motion-prior
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>
#include <hpp/util/debug.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/motion-prior/server.hh>

typedef hpp::corbaserver::motionprior::Server PrecomputationServer;
typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::core::ProblemSolverPtr_t ProblemSolverPtr_t;
typedef hpp::core::ProblemSolver ProblemSolver;
int
main (int argc, char* argv[])
{
  Eigen::internal::set_is_malloc_allowed (true);
  ProblemSolverPtr_t problemSolver = new ProblemSolver;
  CorbaServer corbaServer (problemSolver, argc,
			   const_cast<const char**> (argv), false);
  PrecomputationServer pServer (argc, argv, false);
  pServer.setProblemSolver (problemSolver);

  try {
    corbaServer.startCorbaServer ();
    hppDout (info, "successfully start hpp-corbaserver");
  } catch (const std::exception& exc) {
    hppDout (error, "Faile to start hpp-corbaserver");
  }
  try {
    pServer.startCorbaServer ("hpp", "corbaserver",
				"motion-prior", "precomputation");

    hppDout (notice, "Successfully started corba server for motion-prior precomputation server");
  } catch (const std::exception& exc) {
    hppDout (error, "failed to start corba server for precomputation server");
  }
  corbaServer.processRequest(true);
}

