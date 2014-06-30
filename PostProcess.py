import sys
import os 
#import os.path
#import random
from string import *
import numpy as np
import scipy as sc
import scipy.interpolate as scit
import matplotlib
import pylab as pl
from matplotlib import rc
import vtk

try:
  sys.path.append('/manning/home/xavier/Python/WRL/PySrc')
  import PyReadData as PRD
  print 'external module imported'
except:
  sys.exit('Import ERROR')


datafile="./vtk/snapshot-100.gfs.vtk"
npoints,ncells,nscalars,nvectors,ntensors,Scalarnames, \
	Vectornames,points,datascalar,datavector=PRD.ReadVTKfile(datafile,info=True,UseVectors=False,NewVersion=True)
