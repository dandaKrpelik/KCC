import numpy as np
import matplotlib.pyplot as plt

class _c:##constants
	G = 6.6e-11  ## m3 kg-1 s-2

class co:
	## coordinates
	_dim = 2
	
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
	
	def get_id(self):
		self.__class__.id_counter += 1
		return self.__class__.id_counter
	
	def __init__(self, x, v, M = 1, R = 1):
		self.id = self.get_id()
		self.x = x		
		self.v = v
		self.M = M
		self.R = R

	def d2(self, body2):
		## assume x is co([..,..])
		return  np.sum( np.power( self.x.x - body2.x.x, 2) ) 
		
	def d(self, body2):
		## assume x is co([..,..])
		return  np.sqrt( self.d2(body2) )

	def to(self, body2):##unit direction  vector self->body2
		out = body2.x.x - self.x.x
		d = self.d(body2)
		return out / d

	def move(self, dx):
		self.x.x += dx
		
		
class Bodies:
	def __init__(self):
		self.balls = []	## interacting objects
		


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
	def __init__(self):
		self.x = {}
		self.y = {}
		
		for b in bodies:
			self.x.update( {b.id : [b.x[0]]} )
			self.y.update( {b.id : [b.x[1]]} )
		
	def capture(self):
		for b in bodies:
			self.x[b.id] += [ b.x[0] ]
			self.y[b.id] += [ b.x[1] ]


cap = capturePos()
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
