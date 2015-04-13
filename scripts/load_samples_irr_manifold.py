#/usr/bin/env python
import time
import cPickle as pickle
from irreducible_configuration import IrreducibleConfiguration

ctr=0
pV=[]
with open("04_june_samples.dat", "rb") as f:
    try:
        while ctr < 100:
            pV.append( pickle.load(f) )
            ctr=ctr+1
    except EOFError:
        pass
    except ValueError:
        pass
print "loaded ",ctr," configurations"


#get max and min volume
pvmax = max(pV,key=lambda x:x[0])
pvmin = min(pV,key=lambda x:x[0])

print "max volume ",pvmax[0]," min volume ", pvmin[1]

C= IrreducibleConfiguration(pvmin)
C.display()
