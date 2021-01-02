import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir+'/KCC') 
sys.path.insert(0,parentdir+'/odyn') 

from Missions import *
import math
import Sim


#### Ariser crafts missions
