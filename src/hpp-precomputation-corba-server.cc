#include <iostream>
#include <hpp/util/debug.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/motion-prior/server.hh>

typedef hpp::motion-prior::Server PrecomputationServer;
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
				"motion-prior", "problem");

    hppDout (info, "Successfully started corba server for precomputation server");
  } catch (const std::exception& exc) {
    hppDout (error, "failed to start corba server for precomputation server");
  }
  corbaServer.processRequest(true);
}

