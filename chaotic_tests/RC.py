import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir+'/KCC') 
sys.path.insert(0,parentdir+'/odyn') 

from Missions import *
import math
import Sim

from Optimizer import *
from missionPlannerCareer import *

def get_vessel(ves_name):
	for v in CC.sc.vessels:
		if v.name == ves_name: return v
	return None

def print_vessel_names():
	for v in CC.sc.vessels:
		print(v.name)

v = CC.sc.active_vessel
print_vessel_names()
