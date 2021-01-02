import sys
sys.path.insert(1, '../odyn')

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import bodysim3d as bs
import debug

from mpl_toolkits.mplot3d import Axes3D

data = pd.read_csv('data/default.csv')


def NewtonSim_ascent(t0 = 0.):
	
	#~ bod = bs.Bodies()

	kerbin = bs.Ball([0,0,0], [0,0,0], 5.29e22, 600000)
	kerbin.fixed = True
	
	rate = np.log(1.225/0.288)/10000
	kerbin.atmo = lambda h: 1.225 * np.exp(-h*rate)
	#mun = bs.Ball([-2.373e6 ,0,11.763e6], [ -531.781 , 0.0, -107.279 ] , 9.76e20)
	
	munorbit = bs.CircleOrbit( kerbin , 1.77)
	mun = bs.DepBall(munorbit , 9.76e20)
	
	#print('mun d0 = %f' % (np.linalg.norm(mun.x.x)))
	bodies = [kerbin, mun]

	class capturePos:
		def __init__(self, bodies):
			self.bodies = []
			self.t = [0]
			
			self.x = {}
			self.y = {}
			self.z = {}
			self.thrust = {}
			self.mass = {}
			self.direction = {}
			
			self.tx = {}
			self.ty = {}
			self.tz = {}
			
			self.drag = {}
			
			for b in bodies:
				self.add_body(b)
				
			
		def add_body(self, body):
			b = body
			self.bodies += [b]
			self.x.update( {b.id : [b.x[0]]} )
			self.y.update( {b.id : [b.x[1]]} )
			self.z.update( {b.id : [b.x[2]]} )
			if type(b) is bs.Rocket:
				#self.thrust[b.id] = [ b.total_thrust ]
				self.mass[b.id] = [b.m]
				#self.direction[b.id] = [ b.direction ]
					
				self.drag[b.id] = [b.drag(self.bodies[0])]
					
				#~ t = b.thrust_vector()
				#~ self.tx[b.id] = [t[0]]
				#~ self.ty[b.id] = [t[1]]
				#~ self.tz[b.id] = [t[2]]
			
			
			
		def capture(self):
			for b in self.bodies:
				self.x[b.id] += [ b.x[0] ]
				self.y[b.id] += [ b.x[1] ]
				self.z[b.id] += [ b.x[2] ]
				if type(b) is bs.Rocket:
					#self.thrust[b.id] += [ b.total_thrust(b.height) ]
					self.mass[b.id] += [ b.m ]
					#self.direction[b.id] += [ b.direction ]
					self.drag[b.id] += [b.drag(self.bodies[0])]
					#~ t = b.thrust_vector()
					#~ self.tx[b.id] += [t[0]]
					#~ self.ty[b.id] += [t[1]]
					#~ self.tz[b.id] += [t[2]]

	
	cap = capturePos(bodies)
	return cap

#~ cap = NewtonSim_ascent()

#~ #ker,mun = debug.dstat_mun_dyn(cap)
#~ ker,mun,rck = debug.dstat_rocket_dyn(cap)


def iterate(cap, dt = 1):
	cap.t += [cap.t[-1] + dt]
	
	for b in cap.bodies:
		b.iterate(dt, cap.bodies)



def get_trajectory(t, eles = None):
	cap = NewtonSim_ascent()
	ker,mun,rock = debug.dstat_rocket_dyn(cap, eles)
	
	N = len(t)
	for i in range(1,N):
		dt = t[i] - t[i-1]
		iterate(cap, dt)
		cap.capture()
	
	return cap, rock


def show_trajectory(cap, data = None):
	ker = cap.bodies[0]
	mun = cap.bodies[1]
	rock = cap.bodies[2]
	is_data = not data is None
	
	
	kid = ker.id
	mid = mun.id
	rid = rock.id
	
	
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(cap.x[kid], cap.z[kid], cap.y[kid])
	ax.plot(cap.x[mid], cap.z[mid], cap.y[mid])
	ax.plot(cap.x[rid], cap.z[rid], cap.y[rid])
	ax.scatter([0],[0],[0], s = 50)
	ax.scatter([cap.x[mid][0]], [cap.z[mid][0]],[cap.y[mid][0]],  s = 50)
	ax.scatter([cap.x[rid][0]], [cap.z[rid][0]],[cap.y[rid][0]],  s = 50)
	plt.show(0)

	fig = plt.figure()
	ax = fig.add_subplot(411)
	ax.plot(cap.t, cap.x[rid])
	ax.plot(cap.t, cap.y[rid])
	ax.plot(cap.t, cap.z[rid])
	ax = fig.add_subplot(412)
	ax.plot(cap.t, np.linalg.norm([cap.x[rid], cap.y[rid], cap.z[rid]], axis=0) - ker.R, label = 'sim')
	
	if is_data:
		ax.plot(data['t'], data['alt'], label = 'meas')
		ax.legend()
	ax.axhline(0)
	
	
	ax = fig.add_subplot(413)
	ax.plot(cap.t, cap.mass[rid], label = 'sim')
	if is_data:
		ax.plot(data['t'], data['mass'], label = 'meas')
		ax.legend()
	plt.show(0)

	ax = fig.add_subplot(414)
	ax.plot(cap.t, [ np.linalg.norm(d) for d in cap.drag[rid]] , label = 'sim')
	if is_data:
		ax.plot(data['t'], data['drag'], label = 'meas')
		ax.legend()
	plt.show(0)



	plt.figure()
	plt.plot(cap.x[rid],cap.z[rid])
	plt.scatter([cap.x[rid][0]], [cap.z[rid][0]])
	plt.scatter([0],[0])
	plt.show(0)
		

if __name__ == '__main__':
	t = data['t']
	cap, rock = get_trajectory(t)



#~ dt = 0.05
#~ for i in range(500):
	#~ iterate(dt)
	#~ cap.capture()
#~ dt = 0.2
#~ for i in range(500):
	#~ iterate(dt)
	#~ cap.capture()
#~ dt = 1
#~ for i in range(1500):
	#~ iterate(dt)
	#~ cap.capture()
	
	

	
#~ fig = plt.figure()
#~ ax = fig.add_subplot(111, projection='3d')
#~ ax.plot(cap.x[1], cap.z[1], cap.y[1])
#~ ax.plot(cap.x[2], cap.z[2], cap.y[2])
#~ ax.plot(cap.x[3], cap.z[3], cap.y[3])
#~ ax.scatter([0],[0],[0], s = 50)
#~ ax.scatter([cap.x[2][0]], [cap.z[2][0]],[cap.y[2][0]],  s = 50)
#~ ax.scatter([cap.x[3][0]], [cap.z[3][0]],[cap.y[3][0]],  s = 50)
#~ plt.show(0)

#~ fig = plt.figure()
#~ ax = fig.add_subplot(311)
#~ ax.plot(cap.t, cap.x[3])
#~ ax.plot(cap.t, cap.y[3])
#~ ax.plot(cap.t, cap.z[3])
#~ ax = fig.add_subplot(312)
#~ ax.plot(cap.t, np.linalg.norm([cap.x[3], cap.y[3], cap.z[3]], axis=0) - ker.R)
#~ ax.axhline(0)
#~ ax = fig.add_subplot(313)
#~ ax.plot(cap.t, cap.mass[3])
#~ plt.show(0)

#~ plt.figure()
#~ plt.plot(cap.x[3],cap.z[3])
#~ plt.scatter([cap.x[3][0]], [cap.z[3][0]])
#~ plt.scatter([0],[0])
#~ plt.show(0)
	
	
