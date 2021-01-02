import sys
sys.path.insert(1, '../odyn')

import numpy as np
import matplotlib.pyplot as plt
import bodysim3d as bs

from mpl_toolkits.mplot3d import Axes3D

def NewtonSim_2bodies():
	kerbin = bs.Ball([0,0,0], [0,0,0], 5.29e22, 600000)
	#kerbin.fixed = True
	
	rate = np.log(1.225/0.288)/10000
	kerbin.atmo = lambda h: 1.225 * np.exp(-h*rate)
	mun = bs.Ball([-2.373e6 ,0,11.763e6], [ -531.781 , 0.0, -107.279 ] , 9.76e20)
	
	#munorbit = bs.CircleOrbit( kerbin , 1.77)
	#mun = bs.DepBall(munorbit , 9.76e20)
	
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
			
		def iterate(self, dt):
			self.t += [self.t[-1] + dt]
			for b in self.bodies:
				b.iterate(dt, self.bodies)
				
			
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

def Newton_transfer(alpha = 0):
	cap = NewtonSim_2bodies()
	ker,mun = cap.bodies[0], cap.bodies[1]
	
	ker.rot = 175
	
	rx = ker.R * np.array( [ np.cos(alpha) , 0 , np.sin(alpha) ] )
	vx = ker.rot * np.array( [ -np.sin(alpha) , 0 , np.cos(alpha) ] )
	rock = bs.Rocket( rx, vx )
	
	eles = None
	if eles is None:
		eles = [[85.,60.],[60., 25],[25., 0.]]
	
	cnt1 = bs.Control( rock, ker, eles[0] )
	cnt2 = bs.Control( rock, ker, eles[1] )
	cnt3 = bs.Control( rock, ker, eles[2] )
	
	stage1 = bs.Stage( 16456. , 6150., 1193.16, 250563.22, cnt1 )
	stage2 = bs.Stage( 8646. , 4000., 663.15, 205574.89, cnt2 )
	stage3 = bs.Stage( 2856. , 1000., 43.84, 15125.53, cnt3 )
	rock.add_stage(stage3)
	rock.add_stage(stage2)
	rock.add_stage(stage1)
	
	cap.add_body(rock)
	

def show_trajectory(cap):
	ker = cap.bodies[0]
	mun = cap.bodies[1]
	
	kid = ker.id
	mid = mun.id
	
	cx = {key:np.array(cap.x[key]) for key in cap.x}
	cy = {key:np.array(cap.y[key]) for key in cap.x}
	cz = {key:np.array(cap.z[key]) for key in cap.x}
	
	
	mx = (cx[kid]*ker.M + cx[mid]*mun.M) / (ker.M + mun.M)
	my = (cy[kid]*ker.M + cy[mid]*mun.M) / (ker.M + mun.M)
	mz = (cz[kid]*ker.M + cz[mid]*mun.M) / (ker.M + mun.M)
	
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(cx[kid], cz[kid], cy[kid])
	ax.plot(cx[mid], cz[mid], cy[mid])
	ax.plot( mx , mz, my)
	ax.scatter([0],[0],[0], s = 50)
	ax.scatter([cx[mid][0]], [cz[mid][0]],[cy[mid][0]],  s = 50)
	ax.scatter([cx[kid][0]], [cz[kid][0]],[cy[kid][0]],  s = 50)
	ax.scatter([mx[0]], [mz[0]],[my[0]],  s = 50)
	plt.show(0)

	fig = plt.figure()
	ax = fig.add_subplot(311)
	ax.plot(cap.t, cx[kid])
	ax.plot(cap.t, cx[mid])
	ax.plot(cap.t, cy[kid])
	ax.plot(cap.t, cy[mid])
	ax.plot(cap.t, cz[kid])
	ax.plot(cap.t, cz[mid])
	
	ax = fig.add_subplot(312)
	ax.plot(cap.t, np.linalg.norm([cx[kid]-cx[mid], cy[kid]-cy[mid], cz[kid]-cz[mid]], axis=0), label = 'k-mun')
	ax.plot(cap.t, np.linalg.norm([mx-cx[mid], my-cy[mid], mz-cz[mid]], axis=0), label = 'mun-mid')
	#ax.axhline(ker.R)
	plt.legend()
	plt.show(0)
	
	ax = fig.add_subplot(313)
	ax.plot(cap.t, np.linalg.norm([cx[kid]-mx, cy[kid]-my, cz[kid]-mz], axis=0), label = 'k-mid')#ax.axhline(ker.R)
	plt.legend()
	plt.show(0)
	
	plt.figure()
	plt.plot(cap.x[kid],cap.z[kid], label = 'ker')
	plt.plot(cap.x[mid],cap.z[mid], label = 'mun')
	plt.plot(mx,mz, label = 'mid')
	plt.scatter([cap.x[kid][0]], [cap.z[kid][0]])
	plt.scatter([cap.x[mid][0]], [cap.z[mid][0]])
	plt.scatter([mx[0]], [mz[0]])
	plt.scatter([0],[0], label = '0')
	plt.legend()
	plt.show(0)

	error_v = np.array([ mx[-1]-mx[0], my[-1]-my[0], mz[-1]-mz[0]  ]) / cap.t[-1]
	return error_v

cap = NewtonSim_2bodies()
dt = 25
for i in range(15000):
	cap.iterate(dt)
	cap.capture()


err = show_trajectory(cap)


cap2 = NewtonSim_2bodies()
bodySys = bs.BodySystem([0,0,0], err, cap2.bodies)


dt = 25
for i in range(15000):
	cap2.t += [cap2.t[-1] + dt]
	bodySys.iterate(dt)
	cap2.capture()

err2 = show_trajectory(cap2)
