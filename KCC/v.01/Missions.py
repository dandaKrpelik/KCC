import threading
import time
import numpy as np

from Vessel import *

def drag_coef(flight):
	r = flight.atmosphere_density
	v = np.sum(np.power(flight.velocity,2))
	d = np.sqrt(np.sum(np.power(flight.drag,2)))
	k = 2*d/(r*v)
	return k


class Mission(threading.Thread):
	vessel = None
	flight = None
	orbit = None
	data = {}
	
	def __init__(self, _vessel):
		threading.Thread.__init__(self)
		self.vessel = _vessel
		self.Vessel = Vessel(_vessel)
		
		self.orbit = self.vessel.orbit
		self.flight = self.vessel.flight(self.orbit.body.reference_frame)
		
		self.vessel.auto_pilot.reference_frame = self.vessel.surface_reference_frame
		
		
	def target_min_drag(self):
		v = self.vessel
		f = self.flight
		
		
		
		drag = np.array(f.drag)
		drag /= np.sqrt(np.sum(drag*drag))
		
		v.auto_pilot.target_direction = tuple(-drag)
		return drag
		
	def currentOrbitPosition(self):
		return self.vessel.position(self.orbit.body.reference_frame)

	def while_has_fuel(self, wait_period = .1):
		while (self.Vessel.engines_have_fuel(self.Vessel.stage-1)):
			#self.vessel.auto_pilot.target_direction = self.flight.prograde
			self.target_min_drag()
			self.readData(self.data)
			time.sleep(wait_period)
			
		print('fuel depleted : ', end='')
		self.Vessel.next_stage()
		
	def while_below_apogee(self, wait_period = .1, stage = True):
		while (self.flight.vertical_speed > 0):
			#self.vessel.auto_pilot.target_direction = self.flight.prograde
			#self.target_min_drag()
			self.readData(self.data)
			time.sleep(wait_period)
		
		print('apogee reached : ', end='')	
		if stage: self.Vessel.next_stage()
		

	def keep_retrograde(self):
		while 1:
			self.vessel.auto_pilot.target_direction = self.flight.retrograde
			self.readData(self.data)
			time.sleep(0.1)

	def run_experiments(self):
		for experiment in self.vessel.parts.experiments:
			experiment.run()	
		

class test_mission(Mission):
	def __init__(self, _vessel):
		super().__init__(_vessel)
		self.data = {'pos':[], 'alt':[], 'v_speed':[], 'drag':[]}	


		

	def readData(self,data):
		data['pos'] += [list(self.currentOrbitPosition())]
		data['alt'] += [self.flight.surface_altitude]
		data['v_speed'] += [self.flight.vertical_speed]
		data['drag'] += [drag_coef(self.flight)]
			
	def run(self):
		heading = 90

		vessel = self.Vessel

		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(90, heading)
		
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		self.while_has_fuel()
		vessel.vessel.auto_pilot.target_pitch_and_heading(80, heading)
		
		self.while_has_fuel()
		
		self.while_below_apogee()
		vessel.vessel.control.throttle = .9
		vessel.vessel.auto_pilot.target_pitch_and_heading(30, heading)
		self.while_has_fuel()
		
		self.while_below_apogee()
		self.while_has_fuel()
		self.run_experiments()
		
		vessel.next_stage()
		self.keep_retrograde()
		
class m_orbit1(Mission):
	def __init__(self, _vessel):
		super().__init__(_vessel)
		self.data = {'pos':[], 'alt':[], 'v_speed':[], 'drag':[]}	


	def readData(self,data):
		data['pos'] += [list(self.currentOrbitPosition())]
		data['alt'] += [self.flight.surface_altitude]
		data['v_speed'] += [self.flight.vertical_speed]
		data['drag'] += [drag_coef(self.flight)]
			
	def run(self):
		heading = 90

		vessel = self.Vessel

		vessel.vessel.auto_pilot.engage()
		#vessel.vessel.auto_pilot.target_pitch_and_heading(90, heading)
		
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		self.while_has_fuel()
		#vessel.vessel.auto_pilot.target_pitch_and_heading(80, heading)
		
		self.while_has_fuel()
		
		self.while_below_apogee()
		vessel.vessel.control.throttle = .9
		#vessel.vessel.auto_pilot.target_pitch_and_heading(30, heading)
		self.while_has_fuel()
		
		self.while_below_apogee()
		
		self.run_experiments()
		
		#vessel.next_stage()
		self.keep_retrograde()
		

class test_mission_old(Mission):
	def __init__(self, _vessel):
		super().__init__(_vessel)
		self.orbit = self.vessel.orbit
		self.flight = self.vessel.flight(self.orbit.body.reference_frame)
		self.data = {'pos':[], 'alt':[], 'v_speed':[]}
		
	def readData(self,data):
		data['pos'] += [list(self.currentOrbitPosition())]
		data['alt'] += [self.flight.surface_altitude]
		data['v_speed'] += [self.flight.vertical_speed]		

	def run(self):
		heading = 90
		pitch = 75
		pitch2 = 45
		pitch3 = 10

		vessel = self.vessel
		data = self.data

		vessel.auto_pilot.engage()
		vessel.auto_pilot.target_pitch_and_heading(90, heading)
		start_fuel = vessel.resources.amount('LiquidFuel')

		print('Launch!')
		self.readData(data)
		vessel.control.activate_next_stage()

		time.sleep(2)

		active_engines = []
		for e in vessel.parts.engines:
			if e.active:
				active_engines += [e]

		boosters_active = True
		while boosters_active:
			time.sleep(1)
			self.readData(data)
			boosters_active = False
			for e in active_engines:
				boosters_active = boosters_active or e.has_fuel
			

		vessel.control.activate_next_stage()
		vessel.control.activate_next_stage()

		start_s_fuel = vessel.resources.amount('SolidFuel')

		vessel.auto_pilot.target_pitch_and_heading(pitch,heading)

		while vessel.resources.amount('SolidFuel') > .7 * start_s_fuel:
			time.sleep(1)
			self.readData(data)
				
		vessel.auto_pilot.target_pitch_and_heading(pitch2,heading)
			

		while vessel.resources.amount('SolidFuel') > .4 * start_s_fuel:
			time.sleep(1)
			self.readData(data)
				
		vessel.auto_pilot.target_pitch_and_heading(pitch3,heading)
			
		while vessel.resources.amount('SolidFuel') > 1:
			time.sleep(1)
			self.readData(data)
			
		vessel.control.throttle = 0
		vessel.control.activate_next_stage()

		while vessel.orbit.time_to_apoapsis > 20:
			time.sleep(1)
			self.readData(data)

		vessel.control.throttle = .5

		vessel.auto_pilot.target_pitch_and_heading(20,heading)

		while vessel.resources.amount('LiquidFuel') > start_fuel*0.8:
			time.sleep(1)
			self.readData(data)

		vessel.control.throttle = 0
		while vessel.orbit.time_to_apoapsis > 10:
			time.sleep(1)
			self.readData(data)
		vessel.control.throttle = 0.5

		while vessel.resources.amount('LiquidFuel') > start_fuel*0.6:
			time.sleep(1)
			self.readData(data)
			
		vessel.control.throttle = 0



		#~ while vessel.resources.amount('LiquidFuel') > start_fuel*0.2:
			#~ time.sleep(1)
			#~ readData(data)


		vessel.auto_pilot.disengage()
		vessel.control.sas = True


		while self.flight.vertical_speed > 0:
			print('\r velocity : %f' % radialVelocity(vessel), end = '\r')
			time.sleep(1)
			self.readData(data)
		print('descending')
			
		for experiment in vessel.parts.experiments:
			experiment.run()
