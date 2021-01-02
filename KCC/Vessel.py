from KCC import *
import math

class Pilot:
	active = False
	
	def __init__(self, Vessel, reference_frame = None):
		self.Vessel = Vessel
		self.vessel = Vessel.vessel
	
		if reference_frame is None:
			reference_frame = self.vessel.surface_reference_frame
	
		self.set_reference_frame(reference_frame)
		
	def set_reference_frame(self, rf):
		self.rf = rf
		self.flight = self.vessel.flight(self.rf)
		
	def _engage(self):
		self.active = True
		self.engage()
	def engage(self):
		pass
		
	def _disengage(self):
		self.active = False
		self.disengage()
	def disengage(self):
		pass
		
	def activate_sas(self):
		self.vessel.sas = True
				
	def throttle(self, t):
		self.vessel.control.throttle = t
		
	def steer(self, miss):
		pass

class AutoPilot(Pilot):## krpc def autopilot
	target_dir_tag = None
	
	target = None## (lat, long)
	track = False
	
	def __init__(self, Vessel):
		Pilot.__init__(self, Vessel)
		self.ap = self.vessel.auto_pilot
		self.update_reference_frame(self.rf)
		
		self.ap.deceleration_time = tuple([3,3,3])	## (5,5,5)
		self.ap.time_to_peak = tuple([2,2,2])	##		(3,3,3)
		
	def update_reference_frame(self, rf):
		Pilot.set_reference_frame(self,rf)
		self.ap.reference_frame = self.rf
		
	def engage(self):
		self.ap.engage()
	def disengage(self):
		self.ap.disengage()
	
	def target_head_ele(self, heading, elevation):
		# both in degrees
		
		rad_ele = np.pi*elevation/180
		rad_head = np.pi*heading/180
		
		hor_a = np.cos(rad_ele)
		direction = [np.sin(rad_ele), np.cos(rad_head)*hor_a , np.sin(rad_head)*hor_a ]
		
		self.ap.target_direction = tuple(direction)
	
	def activate_sas(self):
		self.ap.disengage()
		self.vessel.sas = True
		self.ap.sas=True
	
	def wait(self):
		self.ap.wait()
	
	def target_orbital_dir(self, tag, info = True):
		assert tag in self.flight.__dir__()
		
		self.target_dir_tag = tag
		
		self.ap.target_direction = self.flight.__getattribute__(tag)
		
		## dont fall if overshoot apoapsis
		if self.ap.target_direction[0] < 0:
			t = self.ap.target_direction
			self.ap.target_direction = (-t[0],t[1],t[2])
		
		if info: print('pilot targeting '+tag+', i.e. : '+ str(self.ap.target_direction))

	def clear_target(self):
		self.ap.target_roll = np.nan
		self.ap.target_pitch = np.nan

	def target_direction(self,x, rf):
		if x is None or rf is None: return
		self.clear_target()
		self.ap.reference_frame = rf
		self.ap.target_direction = x

	def target_roll(self, roll):
		self.ap.target_roll = roll
	def target_elevation(self, ele):
		self.ap.target_pitch = ele

	def steer(self):
		if self.target_dir_tag:
			self.target_orbital_dir( self.target_dir_tag , info = False)
		if not self.target is None and self.track:
			la = self.target[0] - self.flight.latitude
			lo = self.target[1] - self.flight.longitude
			
			rotation_compensate = 1.2
			heading = 180* ( np.pi/2 - math.atan2( la, la )*rotation_compensate ) / np.pi
			self.ap.target_heading = heading		
			
		

class Engine:
	## assume singe propellant
	def __init__(self, part):
		self.engine = part
		self.propellant_name = self.engine.propellant_names[0]
		self.propellant = self.engine.propellants[0]
		
		self.initial_propellant_amount = self.propellant.current_amount
		
	@property
	def has_fuel(self):
		return self.engine.has_fuel
	@property
	def fuel_ratio(self):
		return self.propellant.current_amount / self.initial_propellant_amount

class Vessel:
	stage= None
	name = 'main'
	
	def __init__(self, vessel):
		self.vessel = vessel
		parts = vessel.parts
		self.parts = parts

		
		orbit = vessel.orbit
		self.flight = vessel.flight(orbit.body.reference_frame)
		
		self.engines = {}
		#~ for x in parts.all:
			#~ self.mass += x.mass
			#~ if not x.engine is None:
				#~ engine = Engine(x)
				#~ decouple_stage = x.decouple_stage
				
				#~ if not decouple_stage in self.engines:
					#~ self.engines.update({decouple_stage : []})
				
				#~ self.engines[decouple_stage] += [engine]
		for x in parts.engines:
			engine = Engine(x)
			decouple_stage = x.part.decouple_stage
				
			if not decouple_stage in self.engines:
				self.engines.update({decouple_stage : []})
				
			self.engines[decouple_stage] += [engine]

		print('vessel loaded; engine stats:')
		est = self.engine_stage_thrust()
		for key in est:
			print('\t'+str(key)+' : T is '+str(est[key]))

		#print('ballistic coef is '+str(self.flight.ballistic_coefficient))


		self.printVesselMeta()


		self.renew_stage()
		print('start stage ' + str(self.stage))
		
	def printVesselMeta(self):
		stage_is = sorted( list(self.engines) , key = lambda x:-x)
		
		for si in stage_is:
			m_tot = 0
			m_t = 0
			m_d = 0
			dm = 0
			th = 0
			
			for p in self.parts.all:
				if p.decouple_stage <= si:
					m_tot += p.mass
					
				if p.decouple_stage == si:
					m_t += p.mass
					m_d += p.dry_mass
				if p.stage > si and p.engine and p.decouple_stage <= si:
					th += p.engine.available_thrust
					dm += p.engine.available_thrust/(10*p.engine.vacuum_specific_impulse)
			
			print('stage dc : %d' % (si))
			print('m_tot %.2f \t m_t %.2f \t m_d %.2f \t dm%.2f \t th%.2f' % (m_tot,m_t, m_d, dm, th))
			print('m0 %.2f \t mf %.2f' % (m_tot, m_t - m_d))

	def engine_stage_thrust(self):
		out = {}
		for key in self.engines:
			ens = self.engines[key]
			t = [0,0]
			for e in ens:
				if not type(e.engine) is CC.sc.Engine: continue
				t[0] += e.engine.max_thrust
				t[1] += e.engine.max_vacuum_thrust
			out[key] = t
		return out
				
	def next_stage(self):
		self.vessel.control.activate_next_stage()
		if self.stage in self.engines:
			self.engines.pop(self.stage)
		self.renew_stage()
		print('start stage ' + str(self.stage))

	def renew_stage(self):
		self.stage = self.vessel.control.current_stage				

	@property
	def orbit_heading(self):
		orbit = self.vessel.orbit
		fl = self.vessel.flight(self.vessel.orbital_reference_frame)
		return fl.direction
	@property
	def surface_dir(self):
		return self.flight.direction
	@property
	def surface_heading(self):
		return (self.flight.heading, self.flight.elevation, self.flight.pitch)
		
	@property
	def orbital_speed(self):
		orbit = self.vessel.orbit
		return orbit.speed

	@property
	def altitude(self):
		orbit = self.vessel.orbit
		return orbit.radius

	@property
	def aoa(self):
		f = self.flight
		return f.angle_of_attack

	@property
	def drag(self):
		f = self.flight
		return f.drag

	@property
	def drag_coef(self):
		f = self.flight
		d = self.drag
		v = self.velocity
		return np.linalg.norm(d)/v**2
		
	@property
	def aero(self):
		f = self.flight
		return f.aerodynamic_force

	@property
	def aero_coef(self):
		f = self.flight
		d = self.aero
		v = self.velocity
		return np.linalg.norm(d)/v**2
		
	
	@property
	def velocity(self):
		v = self.vessel.velocity(self.vessel.orbit.body.reference_frame)
		return np.linalg.norm(v)
		

		
	@property
	def atmo_dens(self):
		f = self.flight
		return f.atmosphere_density
				
	@property
	def mass(self):
		m = 0
		for p in self.parts.all:
			m += p.mass
		return m
	@property
	def thrust(self):
		return self.vessel.thrust
	@property
	def isp(self):
		return self.vessel.specific_impulse
				
	def engines_have_fuel(self, next_stage):
		out = False
		for e in self.engines:
			if e == next_stage:
				for engine in self.engines[e]:
					out = out or engine.has_fuel
		return out

	def stage_fuel_level(self):
		pass
