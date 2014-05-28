from omniORB import CORBA
import CosNaming

from hpp.corbaserver.motion_prior import Precomputation

class CorbaError(Exception):
    """
    Raised when a CORBA error occurs.
    """
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class Client:
  """
  Connect and create clients for hpp-motion-prior library.
  """
  def __init__(self):
    """
    Initialize CORBA and create default clients.
    """
    import sys
    self.orb = CORBA.ORB_init (sys.argv, CORBA.ORB_ID)
    obj = self.orb.resolve_initial_references("NameService")
    self.rootContext = obj._narrow(CosNaming.NamingContext)
    if self.rootContext is None:
        raise CorbaError ('failed to narrow the root context')

    name = [CosNaming.NameComponent ("hpp", "corbaserver"),
            CosNaming.NameComponent ("motionPrior", "precomputation")]
    
    try:
        obj = self.rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound, ex:
        raise CorbaError ('failed to find motionPrior service.')
    try:
        client = obj._narrow (Problem)
    except KeyError:
        raise CorbaError ('invalid service name motionPrior')

    if client is None:
      # This happens when stubs from client and server are not synchronized.
        raise CorbaError (
            'failed to narrow client for service motionPrior')
    self.problem = client
