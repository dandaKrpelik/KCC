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


		
		

def vessel_distance_trajectory(v,w, t1, t0 = None, dN = 1000):
	if t0 is None: 
		t0 = CC.sc.ut	## t1 = dt
		t1 = t0 + t1

	
		
	rf = vo.body.non_rotating_reference_frame
	
	ts = np.zeros(dN)
	ds = np.zeros(dN)
	
	for i in range(dN):
		t = t0 + i*(t1-t0)/dN
		vpos = vo.position_at( t, rf )
		wpos = wo.position_at( t, rf )
		dif = np.subtract( wpos, vpos )
		dist = np.linalg.norm(dif)
		
		ts[i] = t
		ds[i] = dist
	return ts,ds


def vessel_distance_trajectory_o(v,w, t1, t0 = None, dN = 1000):
	vo = v.orbit
	wo = w.orbit
	
	assert vo.body == wo.body
	
	if t0 is None: 
		t0 = CC.sc.ut	## t1 = dt
		t1 = t0 + t1
		
	rf = vo.body.non_rotating_reference_frame
	
	ts = np.zeros(dN)
	ds = np.zeros(dN)
	
	for i in range(dN):
		t = t0 + i*(t1-t0)/dN
		vpos = vo.position_at( t, rf )
		wpos = wo.position_at( t, rf )
		dif = np.subtract( wpos, vpos )
		dist = np.linalg.norm(dif)
		
		ts[i] = t
		ds[i] = dist
	return ts,ds









##################
# kepler
################

def vessel_pos_2d(theta, a, e):#theta as true anomaly
	### in orbital plane with focus in the center of coos and periapsis on the right at (r_p,0)
	r = a * (1-e**2) / (1+e*np.cos(theta))		## dist from focus
	pos = r*np.array([ np.cos(theta), np.sin(theta) ])	
	return pos

def rotmat2d(alpha):
	return np.matrix([[np.cos(alpha), -np.sin(alpha)],[np.sin(alpha), np.cos(alpha)]])

def rotmat3d(alpha, axis):
	A = np.eye(3)
	ind = [i for i in range(3) if not i == axis]
	A[ind,ind] = np.cos(alpha)
	indo = ind.copy()
	ind.reverse()
	A[indo,ind] = np.sin(alpha)
	
	ssign = axis % 2
	A[ indo[ssign] , ind[ssign] ] *= -1
	
	return A

def vessel_pos_3d(theta, a, e, w, i, Tau):###
	## assumes that theta is list
	rotw = rotmat2d(w)	## in rad
	roti = rotmat3d(i,0) #rads ; inclination
	rotA = rotmat3d(Tau, 2)	## rotation of asc node
	
	out = []
	for t in theta:
		x2d = vessel_pos_2d(t,a,e)	
		x2d = rotw.dot(x2d)	## rot periapsis around central body
		x = np.zeros(3)
		x[:2] = x2d
		 #~ = x2d[0]
		#~ x[1]=x2d[1]
		
		x = roti.dot(x)
		x = rotA.dot(x)
		out += [x]
	return out

class KeplerOrbit:
	GM = 6.6e-11 * 5.3e22
	M0 = 0
	
	def __init__(self, t0, x0, a, e, w, i, Om, GM = None):
		self.t0 = CC.sc.ut if t0 is None else t0
		self.x0 = x0
		
		self.a = a
		self.e = e
		self.w = w
		self.i = i
		self.Om = Om
		
		
		if not GM is None:
			self.GM = GM
		
		self.T = 2*np.pi * np.sqrt( self.a**3 /self.GM)	##orb per
		
	@staticmethod
	def get_orbit_from_coos(x, v):
		h = np.cross(x,v)
		
		GM = KeplerOrbit.GM
		
		r = np.linalg.norm(x)
		e = np.cross(v,h)/GM - x/r
		e_scalar = np.linalg.norm(e)
		
		n = np.cross([0,0,1], h)
		
		val = np.arccos( np.dot(e,x) / (r*e_scalar) )
		ta =  2*np.pi - val if np.dot(x,v) < 0 else val
		
		i = np.arccos( h[2] / np.linalg.norm(h) )
		E = 2*np.arctan( np.tan( ta/2 ) / np.sqrt((1+e_scalar)/(1-e_scalar)) )
		
		val = np.arccos(n[0] / np.linalg.norm(n))
		Om = 2*np.pi - val if n[1] < 0 else val
		
		val = np.arccos( n.dot(e) / (np.linalg.norm(n) * e_scalar) )
		w = 2*np.pi - val if e[2] < 0 else val
		
		M = E - e_scalar*np.sin(E)
		
		a = 1./ (2/r - np.linalg.norm(v)**2 / GM)
		
		
		out = KeplerOrbit(None, x, a, e_scalar, w, i, Om)
		out.M0 = M
		out.E = E
		
		return out
		
		
	def at_t(self, t):
		e = self.e
		
		if not '__len__' in t.__dir__(): t = np.array(t)
		t = np.array(t)
		
		Mt = self.get_M_at_t(t)
		E = self.solveKepler(Mt)
		th = 2 * np.arctan( np.sqrt((1+e)/(1-e))*np.tan( E / 2 ) )
		
		return self.x_at_ta(th)
		
	def solveKepler(self, M, n = 100):	#M = E - esin(E)
		E = M
		for i in range(n):
			E = M - self.e * np.sin(E)
		return E
		
	def r_at_ta(self, true_anomaly):
		return self.a*( 1-self.e**2 ) / ( 1 + self.e * np.cos(true_anomaly) )
		
	#~ def calc_M(self, E):
		#~ M = E - self.e * np.cos(E)
		#~ return M
		
	def get_M_at_t(self, t):
		periods_passed = (t - self.t0) / self.T
		M = self.M0 + periods_passed * 2 * np.pi
		return M
		#return ((t - self.t) % self.T ) * (2*np.pi)
		
	def x_at_ta(self, true_anomaly):
		return vessel_pos_3d(true_anomaly, self.a, self.e, self.w, self.i, self.Om)

		
		
class KeplerTrajectory:
	def __init__(self, t, x, orbit):
		self.ts = [t]
		self.orbits = [orbit]
	
	def at(self, t):
		ts = self.ts
		if t < ts[0]:
			return self.orbits[0].x0
		
		oi = 0
		for oi in range(0,len(self.ts)-1):
			if ts[oi] > t: break
		return self.orbits[oi].at(t)
			
		

class OrbitNavigator:
	def __init__(self):
		self.vessel = Vessel(CC.sc.active_vessel)
	
	def track_vessel(self, w):
		vessel = self.vessel.vessel
		vorbit = vessel.orbit
		worbit = w.orbit
	
	
	
	
#### semi-manual
	
	
		## phases


class AimPhase(SingleVesselPhase):
	direction = None
	rf = None
	dt=1
	
	def init(self, miss):
		p = miss.pilots['main']
		p.target_direction(  self.direction , self.rf )
		p.engage()
		p.wait()
		
	def end(self, miss):
		p = miss.pilots['main']
		p.disengage()

class WaitUntilT(SingleVesselPhase):
	ut = 0
	def init(self, miss):
		self.dt = self.ut - CC.sc.ut
		#time.sleep( self.dt )

class BurnManuever(SingleVesselPhase):
	thrust = 1
	direction = None
	rf = None
	
	def init(self, miss):
		SingleVesselPhase.init(self, miss)
		p = miss.pilots['main']
		p.target_direction(self.direction, self.rf)
			
		#time.sleep(self.dt)
		
	def end(self,miss):
		miss.pilots['main'].throttle(0)
		time.sleep(1)

##### missions	
		
def get_execute_manuever_mission(node_i = 0):
	miss = Mission()
	Ves = Vessel(CC.sc.active_vessel)
	miss.add_vessel(Ves)
	miss.add_pilot(AutoPilot( Ves ))
	
	nodes = Ves.vessel.control.nodes
	if len(nodes) == 0: return
	
	node = nodes[node_i]
	t = node.time_to

	dv = node.delta_v
	T = Ves.vessel.available_thrust
	burn_time = Ves.vessel.mass * dv/T
	
	
	miss.update_time = 1
	
	p1 = AimPhase('aiming')
	p1.direction = node.direction(node.reference_frame)
	p1.rf = node.reference_frame
	miss.add_phase(p1)
		
	
			
	p2 = WaitUntilT('waiting for manuever')
	p2.ut = node.ut - burn_time/2
	miss.add_phase(p2)
	


	def addBurnManuever(miss, burn_time):
		miss.addq_change_update_time(.5)
		
		p = BurnManuever('Burn XXX for %.2f sec' % burn_time)
		p.dt = burn_time
		p.direction = node.direction(node.reference_frame)
		p.rf = node.reference_frame
		miss.add_phase(p)
		
	addBurnManuever(miss, burn_time)
	
	
	return miss
		
def get_park_mission(target, dist_ap = 50, t_ap = 320):
	miss = Mission()
	Ves = Vessel(CC.sc.active_vessel)
	rf = Ves.vessel.orbital_reference_frame
	
	if type(target) is CC.sc.Vessel:
		target_vessel = target
	else:
		target_vessel = get_vessel(target)
	if target_vessel is None:
		print('target vessel not found!')
		return
	
	miss.add_vessel(Ves)
	miss.add_pilot(AutoPilot( Ves ))
	miss.update_time = 0.5
	
	#custom params
	miss.target = target_vessel
	
	## 1 - kill rel speed
	#~ direction = -np.array( target_vessel.velocity(rf) )
	#~ dv = np.linalg.norm(direction)
	
	class AimKillVelPhase(AimPhase):
		def init(self, miss):
			v = miss.vessels['main'].vessel
			tar = miss.target
			#self.rf = v.orbital_reference_frame
			self.rf = v.orbit.body.non_rotating_reference_frame
			#~ self.direction = -np.array( tar.velocity(self.rf) )
			self.direction = np.subtract(v.velocity(self.rf), tar.velocity(self.rf) )
			
			AimPhase.init(self,miss)
		
	
	#p1 = AimPhase('P1.1 - aiming opposite of rel velocity')
	p1 = AimKillVelPhase('P1.1 - aiming opposite of rel velocity')
	#p1.direction = direction
	#p1.rf = rf
	miss.add_phase(p1)
	
	#~ T = Ves.vessel.available_thrust
	#~ burn_time = Ves.vessel.mass * dv/T
	
	class BurnAgainstVelManuever(BurnManuever):
		def init(self, miss):
			v = miss.vessels['main'].vessel
			tar = miss.target
			self.rf = v.orbital_reference_frame
			self.direction = -np.array( tar.velocity(self.rf) )
			
			dv = np.linalg.norm(self.direction)
			T = v.available_thrust
			self.dt = v.mass * dv/T
			
			BurnManuever.init(self,miss)
			print('\t initiating a %.3f (s) burn' % self.dt)
			
	#~ p2 = BurnManuever('P1.2 - kill the velocity - burn for %.2f s' % burn_time)
	#~ p2.dt = burn_time
	#~ p2.direction = direction
	#~ p2.rf = rf
	p2 = BurnAgainstVelManuever('P1.2 - execute burn to kill initial relative velocity')
	miss.add_phase(p2)
	
	## 2 - approach  ## get there in t_app sec
	direction = target_vessel.position(rf)
	distance = np.linalg.norm( direction ) - dist_ap
	
	
	## s = .5 T/m t_b^2 + (t_a-t_b)* T/m * t_b
	## t_b^2 *( .5a ) - t_b^2 * a + t_b * (t_a * a) - s = 0
	## t_b^2 *( -.5a ) + t_b * (t_a * a) - distance = 0
	## t_b =  [ -t_a*a  +-  sq(t_a^2 a^2 -2 a distance ) ] / ( -a )
	## t_b = t_a -+ sq( t_a^2 - 2 dist/a )
	acc = Ves.vessel.available_thrust / Ves.vessel.mass
	burn_time = t_ap - np.sqrt( t_ap**2 - 2*distance / acc )
	
	#print('estimated approach burn time set up [for %.1f (s) approach] is %.3f (s)' % (t_ap, burn_time))
	
	class AimTowardsTargetPhase(AimPhase):
		def init(self, miss):
			v = miss.vessels['main'].vessel
			tar = miss.target
			self.rf = v.orbital_reference_frame
			self.direction = tar.position(self.rf) 
	
			AimPhase.init(self,miss)
	
	#~ p1 = AimPhase('P2.1 - aiming towards the target')
	#~ p1.direction = direction
	#~ p1.rf = rf
	p1 = AimTowardsTargetPhase('P2.1 - aiming towards the target')
	miss.add_phase(p1)
	
	class BurnApproachTargetManuever(BurnManuever):
		def init(self, miss):
			v = miss.vessels['main'].vessel
			tar = miss.target
			self.rf = v.orbital_reference_frame
			self.direction = tar.position(self.rf) 
			
			distance = np.linalg.norm( self.direction ) - dist_ap
			acc = v.available_thrust / v.mass
			self.dt = t_ap - np.sqrt( t_ap**2 - 2*distance / acc )
						
			BurnManuever.init(self,miss)
			print('\t initiating a %.3f (s) burn' % self.dt)
	
	#~ p2 = BurnManuever('P2.2 - burn to approach the target')
	#~ p2.dt = burn_time
	#~ p2.direction = direction
	#~ p2.rf = rf
	p2 = BurnApproachTargetManuever('P2.2 - burn to approach the target')
	miss.add_phase(p2)
	
	## 3 - kill the velocity again in the remaining t_approach, i.e. t_a - t_burn
	
	#~ direction = -np.array( target_vessel.velocity(rf) )
	#~ dv = np.linalg.norm(direction)
	
	#~ p1 = AimPhase('P3.1 - aiming opposite of rel velocity')
	#~ p1.direction = direction
	#~ p1.rf = rf
	p1 = AimKillVelPhase('P3.1 - aiming opposite of rel velocity')
	miss.add_phase(p1)
	
	#	when i turn the rel pos change -> scan now
	#~ position = -np.array( target_vessel.position(rf) )
	#~ dist = np.linalg.norm(position) - dist_ap
	
	#~ T = Ves.vessel.available_thrust
	#~ burn_time2 = Ves.vessel.mass * dv/T	## how long to kill vel
	
	## t_tot = t_w + t_b2
	## s = t_tot * dv - 0.5 * T/m * t_b2 ^2 = dist
	## t_w = ( dist + .5 a t_b2^2 - t_b2 * dv ) / dv
	#~ t_wait = ( dist + .5*(T/Ves.vessel.mass) * burn_time2**2 - burn_time2 * dv ) / dv
	
	#~ print('estimated burn to park is %.2f (s). \n total manuever time is %.2f + %.2f + %.2f  =  %.3f (s) --- instead of %.1f (s)' % (burn_time2, burn_time, t_wait, burn_time2, burn_time+t_wait+burn_time2, t_ap))
	
	class WaitUntilApproach(WaitUntilT):
		def init(self,miss):
			self.sas = True
			v = miss.vessels['main'].vessel
			tar = miss.target
			self.rf = v.orbital_reference_frame
			self.direction = tar.position(self.rf) 
			dist = np.linalg.norm( self.direction ) - dist_ap
			
			vel = tar.velocity(self.rf)
			dv = np.linalg.norm(vel)
			
			T = v.available_thrust
			burn_time = v.mass * dv / T
			
			self.dt = ( dist + .5*(T/Ves.vessel.mass) * burn_time**2 - burn_time * dv ) / dv
			print('\t waiting for %.3f (s) for the slow-down burn\n\t\t total approach time is %.3f + %.3f = %.3f (s)' % (self.dt,self.dt,burn_time,self.dt+burn_time))
			self.ut = CC.sc.ut + self.dt
			WaitUntilT.init(self,miss)
	
	#~ pw = WaitUntilT('P3.2 - wait to park well')
	#~ pw.ut = CC.sc.ut + t_wait
	pw = WaitUntilApproach('P3.2 - wait to get closer')
	miss.add_phase(pw)
	
	#~ p2 = BurnManuever('P3.3 kill the velocity - burn for %.2f s' % burn_time)
	#~ p2.dt = burn_time2
	#~ p2.direction = direction
	#~ p2.rf = rf
	p2 = BurnAgainstVelManuever('P3.3 - execute burn to park')
	miss.add_phase(p2)
		
	return miss


miss = None; take_off = True; exe_man = False
if take_off:
	#take-off
	miss = getObserveFlightMission(orbit_height = 100e3)
if exe_man:
	miss = get_execute_manuever_mission(CC.sc.active_vessel,2)

orbin = OrbitNavigator()
v = orbin.vessel
o = v.vessel.orbit
rf = o.body.non_rotating_reference_frame
f = v.vessel.flight(rf)
orb = KeplerOrbit.get_orbit_from_coos(v.vessel.position(rf), f.velocity)

porb = KeplerOrbit(None, v.vessel.position(rf), o.semi_major_axis, o.eccentricity, o.argument_of_periapsis, o.inclination, o.longitude_of_ascending_node)
porb.M0 = o.mean_anomaly

def fit_orbit_transfer( x , orbit_vessel, orbit_target, t1, t2, N=1000):
	# x = [t , PROGRADEdv , NORMALdv, RADIALdv ]
	
	t,P,N,R = x
	
	pos_change = orbit_vessel.at( t )
	dv = orbit_vessel.PNR2dir(P,N,R)
	
	new_orbit = get_orbit_from_xv( pos_change[0], pos_change[1] + dv , t )
	
	dists = []
	for i in range(N):
		t = t1 + (t2-t1)/N 
		x1 = new_orbit.at( t )
		x2 = orbit_target.at( t )
		dif = np.linalg( np.subtract(x1,x2))
		dists += [dif]
		
	return min(dists)

def jump2():
	miss = get_execute_manuever_mission(0)
	miss2 = get_execute_manuever_mission(1)
	
	miss.start()
	miss.join()
	
	miss2.start()
	miss2.join()

