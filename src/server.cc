#include <hpp/util/exception.hh>
#include "hpp/corbaserver/motion-prior/server.hh"
#include "precomputation.impl.hh"

namespace hpp {
  namespace corbaserver {
    namespace motionprior {
      Server::Server (int argc, char *argv[], bool multiThread,
          	    const std::string& poaName) : 
        impl_ (new corba::Server <impl::Precomputation>
               (argc, argv, multiThread, poaName)) {}
      Server::~Server () { delete impl_;}
      void Server::setProblemSolver (ProblemSolverPtr_t problemSolver)
      {
        impl_->implementation ().setProblemSolver (problemSolver);
      }

      /// Start corba server
      void Server::startCorbaServer(const std::string& contextId,
          			  const std::string& contextKind,
          			  const std::string& objectId,
          			  const std::string& objectKind)
      {
        if (impl_->startCorbaServer(contextId, contextKind, objectId, objectKind)
            != 0) {
          HPP_THROW_EXCEPTION (hpp::Exception, "Failed to start corba server.");
        }
      }
    } // namespace motion-prior
  } // namespace corbaserver
} // namespace hpp

