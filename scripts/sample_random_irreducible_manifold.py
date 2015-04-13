#/usr/bin/env python
import time
import os
import cPickle as pickle
from projected_volume import ProjectedVolume
from irreducible_configuration import IrreducibleConfiguration

#Nsamples = 5000000
Nsamples = 100
pv = ProjectedVolume()
filename = 'data-samples.dat'
f = open(filename, 'wb')
fsold = 0

for i in range(0,Nsamples):
  pv.setRandomConfig()
  pv.projectConfigurationUntilIrreducibleConstraint()
  volume = pv.precomputation.getVolume()
  hull = pv.precomputation.getConvexHullCapsules()
  hull = zip(*[iter(hull)]*3)

  projV = [volume, hull, pv.q]

  C=IrreducibleConfiguration(projV)
  C.display()
  pickle.dump( projV, f)
  fs=os.path.getsize(filename) >> 20
  if fs!=fsold:
    print "Samples [",i,"]: filesize: ",fs,"MB"
    fsold=fs
