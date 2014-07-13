# Copyright (c) 2014 LAAS-CNRS
# Author: Andreas Orthey
#
# This file is part of hpp-motion-prior.
# hpp-motion-prior is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-motion-prior is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-motion-prior.  If not, see
# <http://www.gnu.org/licenses/>.

import omniORB
omniORB.updateModule("hpp.corbaserver.motion_prior")
import hpp.corbaserver.motion_prior.precomputation_idl
from client import Client

