import numpy as np
import matplotlib.pyplot as plt
import math

class _c:##constants
	G = 6.67e-11  ## m3 kg-1 s-2

class co:
	## coordinates
	_dim = 3
	
	@property
	def dim(self):
		return self.__class__._dim
	
	def __init__(self,x):
		assert len(x) == self.dim
		self.x = np.array(x, dtype = np.float64)		
	def __getitem__(self, key):
		assert key < self.dim, 'out of dim'
		return self.x[key] 
	

class Ball:
	id_counter = 0
	fixed = False
	atmo = None
	
	dx = None
	dv = None
	
	height = 0
	target_body = None
	
	def get_id(self):
		Ball.id_counter += 1
		return self.__class__.id_counter
	
	def __init__(self, x, v, M = 1, R = 1):
		self.id = self.get_id()
		self.x = co(x)
		self.v = co(v)
		self._M = M
		self.R = R

	def rho(self, h):
		if self.atmo is None:
			return 0
		return self.atmo(h)

	@property
	def M(self):
		return self._M

	def d2(self, body2):
		## assume x is co([..,..])
		#return np.power(self.d(body2),2)
		return sum( np.power( self.x.x - body2.x.x, 2) ) 
		
	def d(self, body2):
		## assume x is co([..,..])
		return  np.linalg.norm(self.x.x - body2.x.x)
		#return  np.sqrt( self.d2(body2) )

	def to(self, body2):##unit direction  vector self->body2
		out = body2.x.x - self.x.x
		#return out
		
		#d = self.d(body2)
		#return out / d
		d = np.linalg.norm(out)
		assert d > 1e-5
		return out / d
		
	def rot_angle(self, target):
		r = self.x.x - target.x.x
		
		#~ out = math.atan2(r[0],r[1])
		#~ return np.pi*out/180
		
		s = np.sign(r[1])
		
		if s == 0:
			return np.pi/2 if r[0] < 0 else -np.pi/2
		elif s>0:	
			return np.arctan(-r[0]/r[1])
		else:
			return np.pi + np.arctan(r[0]/r[1])

	def newton(self, dt, balls):
		dv = np.zeros(co._dim)
		for ball in balls:
			
			if self.id == ball.id : continue
			dist2 = self.d2(ball)
			a =  ( self.to(ball) ) * ( _c.G * ball.M / dist2 )
			dv += a
			
		return dv

	def iterate(self, dt, balls):
		self.dx = np.array(self.v.x)
		self.dv = np.zeros(co._dim)
		if not self.fixed:
			self.dv += self.newton(dt, balls)
			
		self.x.x += self.dx * dt + 0.5 * self.dv * dt * dt
		self.v.x += self.dv * dt	
	
class CircleOrbit:
	def __init__(self, focus, w0 = 0, A = 12e6, P = 141115.4 ):
		self.A = A# apoapsis
		self.P = P#period
		self.focus = focus
		self.w0 = w0
	
		self.omega = 2*np.pi / P
	
	def x(self, t):
		mean_anomaly = self.w0 + ( self.omega * t  % 2*np.pi)
		out = [ self.A * np.cos(mean_anomaly), 0 , self.A * np.sin(mean_anomaly) ]
		return out 
	def v(self, t):
		mean_anomaly = self.w0 + ( self.omega * t  % 2*np.pi)
		x = self.x(t)
		r = np.linalg.norm(x)
		v = self.omega * r
		out = [ -v*np.sin(mean_anomaly), 0, -v*np.cos(mean_anomaly) ]
		return out
		
		
	
class DepBall(Ball):
	def __init__(self, orbit, M=1, R=1):
		super().__init__(orbit.x(0), orbit.v(0), M, R)
		self.orbit = orbit
		
		self.t = 0.
		self.x = co(orbit.x(0))
		self.v = co(orbit.v(0))
	
	
	def iterate(self, dt, balls):
		self.t += dt
		self.x.x = self.orbit.x(self.t)
		self.v.x = self.orbit.v(self.t)
	
class Atmosphere:
	def __init__(self, h0 = 6e5, h1 = 8e7, p0 = 101325 , H = 5600):
		self.h0 = h0
		self.h1 = h1
		
		self.H = H
		self.p0 = p0
		
	def rho(self, height):
		#if height < self.h0/2: return np.nan
		if height > self.h1: return 0
		
		return 1.225*np.exp(-(height - self.h0)/7e3)





class Control:
	def __init__(self, rck, body, ele):
		self.rck = rck
		self.body = body
		self.ele = ele
		
	@property
	def direction(self):
		rck = self.rck
		
		head = 90.
		if type(self.ele) is list:
			ele = self.ele[0] + self.ele[1]*( rck.stages[rck.n_stage].m0 - rck.m ) / (rck.stages[rck.n_stage].m0 - rck.stages[rck.n_stage].mf)
		else:
			ele = self.ele
			
		out_dir = self.body.to(rck)
		v_dir = self.rck.v.x
		v = np.linalg.norm(v_dir)
		for_dir = v_dir/v - out_dir.dot(v_dir/v) * v_dir/v
		
		eled = np.deg2rad(ele)
		out = np.sin(eled ) * out_dir + np.cos( eled) * for_dir
		
		return out
		
class Stage:
	def __init__(self, m0, mf, dm , thrust_max, control):
		self.m0 = m0
		self.mf = mf
		self.dm = dm
		self.thrust_max = thrust_max
		self.control = control
		
		self.mdry = m0-mf
		
class Rocket(Ball):
	def __init__(self,x,v):
		super().__init__( x,v, -1, 0)
		self.n_stage = -1
		self.stages = []
		
		self.m = 0
		self.C = 0.4
		
	@property
	def M(self):
		return self.m
		
	def add_stage(self, stage):
		self.stages += [ stage ]
		self.n_stage += 1
		self.m = stage.m0
	
	def next_stage(self):
		self.n_stage -= 1
		if self.n_stage >= 0:
			self.m = self.stages[self.n_stage].m0
	
	def drag(self, body):
		v = self.v.x
		speed = np.linalg.norm(v)
		
		alt = body.d(self) - body.R
		
		out = -0.5 * v*speed *self.C*body.rho( alt )
		return out 
	
	def newton(self, dt, balls):
		dv = Ball.newton(self,dt,balls)
		dv += self.drag(balls[0] )/ self.m
		#print('rocket newton')
		
		if self.n_stage >= 0:
			stage = self.stages[self.n_stage]
			dv +=  stage.thrust_max * stage.control.direction / self.m 
			self.m -= stage.dm/10 * dt
			if self.m < stage.mdry:
				self.next_stage()
		
		return dv
		
	def iterate(self, dt, balls):
		dx = np.array(self.v.x)
		dv = np.zeros(co._dim)
		if not self.fixed:
			dv += self.newton(dt, balls)
			
		self.x.x += dx * dt + 0.5 * dv * dt * dt
		self.v.x += dv * dt		
	
		
class BodySystem(Ball):
	def __init__(self, x, v, balls = []):
		super().__init__( x,v, sum([ball.M for ball in balls]), 0)
		self.balls = balls	## interacting objects
		for b in balls:
			b.x.x -= x
			b.v.x -= v
				
	def iterate(self, dt):
		balls = self.balls
		for b in balls:
			b.iterate(dt, balls)
		
		dv = self.v.x * dt
		self.x.x += dv
		#~ for b in balls:
			#~ b.x.x -= dv
			

class capturePos:
	def __init__(self, bodies):
		self.bodies = bodies
		
		self.x = {}
		self.y = {}
		self.thrust = {}
		self.mass = {}
		self.direction = {}
			
		self.tx = {}
		self.ty = {}
			
		self.n = 0
			
		for b in bodies:
			self.x.update( {b.id : [b.x[0]]} )
			self.y.update( {b.id : [b.x[1]]} )
			if type(b) is Rocket:
				self.thrust[b.id] = [ b.total_thrust(b.height) ]
				self.mass[b.id] = [b.M]
				self.direction[b.id] = [b.direction]
					
				t = b.thrust_vector()
				self.tx[b.id] = [t[0]]
				self.ty[b.id] = [t[1]]
			
	def capture(self):
		for b in self.bodies:
			self.x[b.id] += [ b.x[0] ]
			self.y[b.id] += [ b.x[1] ]
			if type(b) is Rocket:
				self.thrust[b.id] += [ b.total_thrust(b.height) ]
				self.mass[b.id] += [ b.M ]
				self.direction[b.id] += [ b.direction ]
					
				t = b.thrust_vector()
				self.tx[b.id] += [t[0]]
				self.ty[b.id] += [t[1]]
		self.n += 1


if __name__ == '__main__':

	sun = Ball(co([0,0]), co([0,0]), 2e30)
	earth = Ball(co([1.5e11,0]), co([ 0 , 3e4 ]) , 6e24)
	moon = Ball(co([1.5e11,4e8]), co([0 - 1e3, 3e4 ]) , 7.3e22)

	bodies = [sun, earth, moon]

	def iterate(dt = 1):
		for b in bodies:
			dv = np.zeros(co._dim)
			for c in bodies:
				if b.id == c.id : continue
				
				dist = b.d2(c)
				a =  ( b.to(c) ) * ( _c.G * c.M / dist )
				dv += a
			
			b.x.x += b.v.x * dt
			b.v.x += dv * dt
		
		

	class capturePos:
		def __init__(self, bodies):
			self.bodies = bodies
			
			self.x = {}
			self.y = {}
			self.thrust = {}
			self.mass = {}
			self.direction = {}
			
			self.tx = {}
			self.ty = {}
			
			self.n = 0
			
			for b in bodies:
				self.x.update( {b.id : [b.x[0]]} )
				self.y.update( {b.id : [b.x[1]]} )
				if type(b) is Rocket:
					self.thrust[b.id] = [ b.total_thrust(b.height) ]
					self.mass[b.id] = [b.M]
					self.direction[b.id] = [b.direction]
					
					t = b.thrust_vector()
					self.tx[b.id] = [t[0]]
					self.ty[b.id] = [t[1]]
			
		def capture(self):
			for b in self.bodies:
				self.x[b.id] += [ b.x[0] ]
				self.y[b.id] += [ b.x[1] ]
				if type(b) is Rocket:
					self.thrust[b.id] += [ b.total_thrust(b.height) ]
					self.mass[b.id] += [ b.M ]
					self.direction[b.id] += [ b.direction ]
					
					t = b.thrust_vector()
					self.tx[b.id] += [t[0]]
					self.ty[b.id] += [t[1]]
			self.n += 1



	cap = capturePos(bodies)
	dt = 1250
	for i in range(10000):
		iterate(dt)
		cap.capture()
		
		
	plt.figure()
	plt.plot( cap.x[ earth.id ] , cap.y[ earth.id ] )
	plt.plot( cap.x[ moon.id ] , cap.y[ moon.id ] )
	plt.show(0)

	plt.figure()
	plt.plot( [cap.x[ earth.id ][i] - cap.x[ moon.id ][i] for i in range(len(cap.x[1]))], [cap.y[ earth.id ][i] - cap.y[ moon.id ][i] for i in range(len(cap.y[1]))] )
	plt.show(0)
