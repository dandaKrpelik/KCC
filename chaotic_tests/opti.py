import sys
sys.path.insert(1, '../odyn')
from Optimizer import *

import stats

data = stats.data




def getTraj(xin):
	stages = 3
	xeles = []
	
	x = xin.copy()
	x[:stages] = abs(x[:stages])
	sx = np.cumsum(x[:stages])
	sx /= sx[-1]
	xeles += [ 90. * (1. - sx[i]) for i in range(stages) ]
	eles = []
	for i in range(stages):
		last = 90 if i == 0 else eles[-1][1]
		eles += [ [last, xeles[i]] ]
		
	print(eles)
	cap, rock = stats.get_trajectory(data['t'], eles)
	return cap, rock
	
def fit(xin):
	cap, rock = getTraj(xin)
	ker = cap.bodies[0]
	
	simalt = np.linalg.norm([cap.x[rock.id], cap.y[rock.id], cap.z[rock.id]], axis=0) - ker.R
	mesalt = data['alt']
	
	w = np.linspace(0,1,len(simalt))
	w = np.power(w,2)
	
	out = sum(w * np.power( simalt - mesalt ,2))/sum(w)
	return out
	
def fit2(xin):
	cap, rock = getTraj(xin)
	ker = cap.bodies[0]
	
	r = rock.x.x
	v = rock.v.x
	
	d = np.linalg.norm(r)
	
	mu = ker.M * 6.67e-11
	vid = np.sqrt( mu / d )
	
	vproj = v - v * r/d
	vprojnorm = np.linalg.norm(vproj)
	
		
	out = (vid - vprojnorm)**2
	return out
	

	
de = DE(fit2, 15, 3, [[0,1],[0,5],[0,1]])

