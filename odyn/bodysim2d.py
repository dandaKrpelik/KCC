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
		self.__class__.id_counter += 1
		return self.__class__.id_counter
	
	def __init__(self, x, v, M = 1, R = 1):
		self.id = self.get_id()
		self.x = x		
		self.v = v
		self._M = M
		self.R = R

	@property
	def M(self):
		return self._M

	def d2(self, body2):
		## assume x is co([..,..])
		return np.power(self.d(body2),2)
		#np.sum( np.power( self.x.x - body2.x.x, 2) ) 
		
	def d(self, body2):
		## assume x is co([..,..])
		return  np.linalg.norm(self.x.x - body2.x.x)
		#return  np.sqrt( self.d2(body2) )

	def to(self, body2):##unit direction  vector self->body2
		out = body2.x.x - self.x.x
		return out
		#d = self.d(body2)
		#return out / d
		d = np.linalg.norm(out)
		if d < 1e-5: return 0
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


	def iterate(self, dt, balls):
		self.dx = np.array(self.v.x)
		self.dv = np.zeros(co._dim)
		if not self.fixed:
			dv = np.zeros(co._dim)
			for ball in balls:
			
				if self.id == ball.id : continue
				dist2 = self.d2(ball)
				a =  ( self.to(ball) ) * ( _c.G * ball.M / dist2 )
				dv += a
			
			self.dv += dv
			
	def update(self, dt):
		self.x.x += self.dx * dt + 0.5 * self.dv * dt * dt
		self.v.x += self.dv * dt	
	
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

class Thruster:
	change_thrust = False
	active = False
	atmo = None
	on_halt = False	## paused engine
	
	next_stage = []
	observers = []	## decoupled stage payload mass
	name = ''
	
	M = 0
	def __init__(self, thrust, isp, M_tank=0, M_fuel=0, multi = 1):
		self._thrust = thrust
		self._isp = isp
		
		if type(thrust) is list or type(isp) is list:
			self.change_thrust = True	## interpolate actual isp and T
			get_list = lambda x: x if type(x) is list else [x,x]
			self._thrust = get_list(thrust)
			self._isp = get_list(isp)
			
		self.M_tank = M_tank
		self.M_fuel = M_fuel
		self.multi = multi ## if there are more iid engines
		
		self.init()
	
	h0 = 6e5
	h1 = 8e7
	
	def thrust(self, h = h0):
		return self._c_thrust( h ) * self.multi
	def _c_thrust(self, h = h0):
		if not self.active: return 0
		
		t = self._thrust		
		if not self.atmo: return t[1]
		
		h0 = self.atmo.h0
		h1 = self.atmo.h1
		
		if self.change_thrust:
			if h < h0: return t[0]
			if h > h1: return t[1]
			return (h-h0) * (t[1]-t[0])/(h1-h0) + t[0]
		return t
		
	def isp(self, h = h0):
		isp = self._isp
		
		if not self.atmo: return isp[1]
		
		h0 = self.atmo.h0
		h1 = self.atmo.h1
		
		if self.change_thrust:
			if h < h0: return isp[0]
			if h > h1: return isp[1]
			return (h-h0) * (isp[1]-isp[0])/(h1-h0) + isp[0]
		return isp
			
	
	def init(self):
		self.M = self.multi * (self.M_tank + self.M_fuel)
		
	#~ @property
	#~ def M(self):
		#~ return self._M
		
	def activate(self):
		self.active = True
		self.on_halt = False
	def deactivate(self):
		if self.active:
			self.on_halt = True
			self.active = False
		
	def decouple(self):
		print('decoupling')
		self.M = 0
		self.active = False
		
		for e in self.next_stage:
			e.activate()
		for e in observers:
			e.decouple()
		
	def dm(self, h = h0):
		if not self.active: return 0
	
		g = 9.81
		
		T = self.thrust(h)
		I = self.isp(h)
		return T/(I*g)
	
	def burn(self, dt, height, rocket = None):
		out = self.thrust(height)
		if out > 0:
			self.M -= self.dm(height) * dt
			if self.M < self.multi*self.M_tank:
				self.M = self.multi*self.M_tank
				#self.active = False
				self.decouple()
			
				if not rocket is None:
					print('at x: '+str(rocket.x.x)+'  , v: '+str(rocket.v.x))
			
		return out
		
class Rocket(Ball):
	#~ delta_m = 0
	#~ thrust = 0
	direction = 0 ## in [0, 2pi] - deviation from outward
	dir_change = 3
	target_body = None
	M_payload = 0
	BC = 0.05 * 2
	
	dd_v_speed_lim = 100
	dd_1 = np.pi/20
	
	stage = 0
			
	def __init__(self, x,v,target=None):
		super().__init__(x,v)
		self.target_body = target
		
		self.engines = []
	
	def init(self):
		for e in self.engines:
			e.init()
	
	def set_target_body(self, tb):
		self.target_body = tb
		for e in self.engines:
			e.atmo = tb.atmo
		
	def frame_angle(self):
		if self.target_body is None: return 0
		return self.target_body.rot_angle(self)
		
	@property
	def M(self):
		out = 0
		for e in self.engines:
			out += e.M
		return out + self.M_payload
		
		
	def total_thrust(self, height):
		height = np.linalg.norm(self.x.x)
		thrust = 0
		for e in self.engines:
			thrust += e.thrust(height)		
		return thrust
	
	@property
	def height(self):
		return np.linalg.norm(self.x.x)
	
	def thrust_vector(self):
		height = self.height
		dv = np.zeros(co._dim)
		thrust = 0
		for e in self.engines:
			thrust += e.thrust(height)
			
		if thrust > 0:
			thrust_angle = self.direction#*np.pi/2
			pos_angle = np.pi/2-math.atan2(self.x[0],self.x[1])
			
			angle = thrust_angle + pos_angle
			T_abs = thrust * np.array( [ np.cos(angle) , np.sin(angle) ])
			
					
			dv += T_abs
		return dv		
	
	
	def iterate(self, dt, balls):
		Ball.iterate(self,dt, balls)
		self.dv += self.thrust_vector() / self.M
		
		
		#drag
		rho = self.target_body.atmo.rho(self.height)
		self.dv -= .5 * self.BC * np.linalg.norm(self.v.x) * self.v.x / self.M
	
	@property
	def altitude(self):
		return self.height - self.target_body.R
	
	def update(self, dt):
		Ball.update(self, dt)
		
		
		if self.stage == 0:
			if self.altitude > 80000:
				self.stage = 1
				for e in self.engines:
					e.deactivate()
				self.direction = np.pi / 3
			elif self.altitude > 60000:
			#~ if self.altitude > 50000:
				ddir = np.pi* self.dir_change /180
				if self.direction < (np.pi / 3 +np.pi/64):
					self.direction += ddir * dt
			elif self.altitude > 25000:
				ddir = np.pi* self.dir_change /180
				if self.direction < (np.pi / 4 +np.pi/64):
					self.direction += ddir * dt
			elif self.altitude > 2500:
				ddir = np.pi* self.dir_change /180
				if self.direction < (np.pi / 8):
					self.direction += ddir * dt
			elif self.direction == 0 and np.linalg.norm(self.v.x) > self.dd_v_speed_lim:
				self.direction= self.dd_1
		
		if self.stage == 1:
			if np.linalg.norm( self.target_body.to(self).dot(self.v.x) ) < 5e1:
				self.stage = 2
				for e in self.engines:
					if e.on_halt:
						e.activate()
			
		for e in self.engines:
			e.burn(dt, self.height, self)
	
		
		
class Bodies:
	def __init__(self, balls = []):
		self.balls = balls	## interacting objects
				
	def iterate(self, dt):
		
		balls = self.balls
		for b in balls:
			b.iterate(dt, balls)
			b.update(dt)
			

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
	bod = Bodies()

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
