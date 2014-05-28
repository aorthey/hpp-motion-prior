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
#ifndef HPP_PRIOR_CORBA_SERVER_HH
# define HPP_PRIOR_CORBA_SERVER_HH

# include <hpp/corba/template/server.hh>
# include <hpp/corbaserver/motion-prior/fwd.hh>

namespace hpp 
{
  namespace corbaserver 
  {
    namespace motionprior 
    {
      namespace impl 
      {
        class Precomputation;
      }// namespace impl
            
      class Server
      {
      public:
        Server (int argc, char *argv[], bool multiThread = false,
                const std::string& poaName = "child");
        ~Server ();
        /// Set planner that will be controlled by server
        void setProblemSolver (ProblemSolverPtr_t problemSolver);

        /// Start corba server
        /// Call hpp::corba::Server <impl::Precomputation>::startCorbaServer
        void startCorbaServer(const std::string& contextId,
          		    const std::string& contextKind,
          		    const std::string& objectId,
          		    const std::string& objectKind);
      private:
        corba::Server <impl::Precomputation>* impl_;
      };
    } // namespace motionprior
  }//namespace corbaserver
} // namespace hpp
#endif
