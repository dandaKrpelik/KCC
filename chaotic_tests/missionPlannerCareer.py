import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir+'/KCC') 

from KCC import *

class M_measure(Mission):
	def __init__(self, _vessel):
		super().__init__(_vessel)
		self.data = {'t':[], 'pos':[], 'alt':[], 'v_speed':[], 'drag':[], 'dens':[]}	
		self.t0 = None

	def readData(self,data):
		data['t'] += [CC.sc.ut - self.t0]
		data['pos'] += [list(self.currentOrbitPosition())]
		data['alt'] += [self.flight.surface_altitude]
		data['v_speed'] += [self.flight.vertical_speed]
		data['drag'] += [drag_coef(self.flight)]
		data['dens'] += [self.flight.atmosphere_density]
			
	def keep_logging(self,t):
		t0 = CC.sc.ut
		while CC.sc.ut - t0 < t:
			self.readData(self.data)
			time.sleep(0.1)
			
	def while_below_apogee(self, wait_period = 0.1):
		while (self.flight.vertical_speed > 0):
			#self.vessel.auto_pilot.target_direction = self.flight.prograde
			self.target_min_drag()
			self.readData(self.data)
			time.sleep(wait_period)
		print('apogee reached : ', end='')	
					
	def run(self):
		heading = 320
		vessel = self.Vessel

		vessel.vessel.sas=True
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(85, heading)
		
		self.t0 = CC.sc.ut
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		
		self.keep_logging(2)
		vessel.vessel.auto_pilot.target_pitch_and_heading(65, heading)
		self.keep_logging(3)
		vessel.vessel.auto_pilot.target_pitch_and_heading(35, heading)
		
		self.while_below_apogee()
		vessel.next_stage()
		
		self.run_experiments()
		vessel.next_stage()
		
		#vessel.vessel.auto_pilot.disengage()
		
		self.keep_retrograde()
		
		
class Leaper_O(Mission):
	def __init__(self, _vessel):
		super().__init__(_vessel)
		self.data = {'t':[], 'pos':[], 'alt':[], 'v_speed':[], 'drag':[], 'dens':[]}	
		self.t0 = None
		self.dlogt = 0.1

	def readData(self,data):
		t = CC.sc.ut
		
		if self.t0 is None:
			self.t0 = t
		else:
			if t - data['t'][-1] < self.dlogt : return
		
		
		data['t'] += [CC.sc.ut - self.t0]
		data['pos'] += [list(self.currentOrbitPosition())]
		data['alt'] += [self.flight.surface_altitude]
		data['v_speed'] += [self.flight.vertical_speed]
		data['drag'] += [drag_coef(self.flight)]
		data['dens'] += [self.flight.atmosphere_density]
			
		self.report()
			
	def keep_logging(self,t, wait_period = 0.1):
		t0 = CC.sc.ut
		while CC.sc.ut - t0 < t:
			self.readData(self.data)
			time.sleep(wait_period)
			
	def while_below_apogee(self, wait_period = 0.1, mindrag = False):
		while (self.flight.vertical_speed > 0):
			#self.vessel.auto_pilot.target_direction = self.flight.prograde
			if mindrag: self.target_min_drag()
			self.readData(self.data)
			time.sleep(wait_period)
		print('apogee reached : ', end='')	

	def while_has_fuel(self, wait_period = .1, mindrag = False):
		while (self.Vessel.engines_have_fuel(self.Vessel.stage-1)):
			#self.vessel.auto_pilot.target_direction = self.flight.prograde
			if mindrag: self.target_min_drag()
			self.readData(self.data)
			time.sleep(wait_period)
			
		print('fuel depleted : ', end='')
		
	def report(self):
		#print('\rtarget dir is '+str(self.vessel.auto_pilot.target_direction), end = '\r')
		pass
			
	def run(self):
		heading = 90
		vessel = self.Vessel

		#vessel.vessel.sas=True
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(90, heading)
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		## 1st 4 engines
		self.keep_logging(10)
		
		self.dlogt = 0.5
		#vessel.vessel.auto_pilot.target_pitch_and_heading(75, heading)
		self.while_has_fuel( mindrag = False )
		vessel.next_stage() # decouple
		
		## big booster
		self.dlogt=2
		vessel.vessel.auto_pilot.target_pitch_and_heading(80, heading)
		#self.while_below_apogee(1, mindrag=False)
		vessel.next_stage() # start big booster
		self.while_has_fuel( mindrag = False )
		vessel.next_stage()	#	decouple
		
		
		## liquid
		self.dlogt=10
		vessel.vessel.auto_pilot.target_direction = self.flight.retrograde
		self.while_below_apogee(5, mindrag = False)
		self.run_experiments()


#		vessel.next_stage()
		#vessel.vessel.auto_pilot.disengage()
		
		self.keep_retrograde()	
		

class Leaper(Mission):
	def __init__(self, _vessel):
		super().__init__(_vessel)
		self.data = {'t':[], 'pos':[], 'alt':[], 'v_speed':[], 'drag':[], 'dens':[]}	
		self.t0 = None
		self.dlogt = 0.1
		
		
	def readData(self,data):
		t = CC.sc.ut
		
		if self.t0 is None:
			self.t0 = t
		else:
			if t - data['t'][-1] < self.dlogt : return
		
		
		data['t'] += [CC.sc.ut - self.t0]
		data['pos'] += [list(self.currentOrbitPosition())]
		data['alt'] += [self.flight.surface_altitude]
		data['v_speed'] += [self.flight.vertical_speed]
		data['drag'] += [drag_coef(self.flight)]
		data['dens'] += [self.flight.atmosphere_density]
			
		self.report()
			
	def keep_logging(self,t, tasks, wait_period = 0.1):
		print('waiting for '+str(t)+' sec')
		t0 = CC.sc.ut
		while CC.sc.ut - t0 < t:
			
			for task in tasks:
				task[0](*task[1])
			
			
			self.readData(self.data)
			time.sleep(wait_period)
		print('wait finished')
			
	def while_below_apogee(self, tasks, wait_period = 0.1):
		print('waiting for apoapsis')
		while (self.flight.vertical_speed > 0):
			
			for task in tasks:
				out = task[0](*task[1])
				if out:
					CC.KSPconnection.drawing.add_direction(out , self.vessel.surface_reference_frame )
					CC.KSPconnection.drawing.add_direction(out , self.orbit.body.reference_frame )
				
			
			self.readData(self.data)
			time.sleep(wait_period)
		print('apogee reached : ', end='')	

	def while_general(self, condition, tasks, wait_period = .1 , info=None):
		if info is None: info = 'general condition '+str(condition)
		print(info)
		while(condition()):
			for task in tasks:
				out = task[0](*task[1])	
			self.readData(self.data)
			time.sleep(wait_period)
		print('general condition finished')							

	def while_has_fuel(self, tasks, wait_period = .1):
		print('burning all fuel in stage')
		while (self.Vessel.engines_have_fuel(self.Vessel.stage-1)):
			
			for task in tasks:
				task[0](*task[1])
			
			self.readData(self.data)
			time.sleep(wait_period)
			
		print('fuel depleted : ', end='')
		
	def while_resource( self, tag , tasks , lim = 0 , wait_period = 0.1 ):
		start_val = self.Vessel.vessel.resources.amount(tag)
		print('burning '+tag+' until '+str(lim*100)+'%%  .. ie %.1f from %.1f' % (lim*start_val , start_val))
		
		while (self.Vessel.vessel.resources.amount(tag)/start_val > lim):
			
			for task in tasks:
				task[0](*task[1])
			
			self.readData(self.data)
			time.sleep(wait_period)
			
		print(tag + ' down on '+str(lim*100)+'%')				
			
		
	def set_auto_direction(self, symdir):
		
		#self.sync_AP_ref_frame()
		
		if symdir == 'prograde':
			self.Vessel.vessel.auto_pilot.target_direction = CC.sc.transform_direction( 
					self.flight.prograde, 
					self.orbit.body.reference_frame,
					self.vessel.surface_reference_frame
					)
		elif symdir == 'retrograde':
			self.Vessel.vessel.auto_pilot.target_direction =  CC.sc.transform_direction( 
					self.flight.retrograde,
					self.orbit.body.reference_frame,
					self.vessel.surface_reference_frame
					)
		elif symdir == 'radial':
			self.Vessel.vessel.auto_pilot.target_direction =  CC.sc.transform_direction( 
					self.flight.radial,
					self.orbit.body.reference_frame,
					self.vessel.surface_reference_frame
					)
		elif symdir == 'normal':
			self.Vessel.vessel.auto_pilot.target_direction =  CC.sc.transform_direction( 
					self.flight.normal,
					self.orbit.body.reference_frame,
					self.vessel.surface_reference_frame
					)
		
	def report(self):
		#print('\rtarget dir is '+str(self.vessel.auto_pilot.target_direction), end = '\r')
		pass
			
	def run_O(self):
		heading = 170
		vessel = self.Vessel

		#vessel.vessel.sas=True
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(90, heading)
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		## 1st 4 engines
		self.keep_logging(5,[])
		
		self.dlogt = 0.5
		#vessel.vessel.auto_pilot.target_pitch_and_heading(75, heading)
		self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (88, heading) ]] )
		
		self.keep_logging(2,[])
		
		vessel.next_stage() # decouple
		
		self.keep_logging(5,[])
		## big booster
		self.dlogt=2
		vessel.next_stage() # start big booster
		self.keep_logging(2,[])
		
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (80, heading) ]] , 0.8, 5 )
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (70, heading) ]] , 0.6, 5 )
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (60, heading) ]] , 0.3, 5 )
	#	self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (50, heading) ]] )
		self.while_has_fuel( [[ self.target_min_drag , () ]] )
		vessel.next_stage()	#	decouple
		
		
		## liquid
		self.keep_logging(1,[])
		start_fuel = vessel.vessel.resources.amount('LiquidFuel')
		self.dlogt=10
		vessel.vessel.control.throttle = .1
		vessel.next_stage() # start the engine
		self.while_resource( 'LiquidFuel' , [[ self.set_auto_direction , ('prograde',) ]] , 0.5, 5 )
		vessel.vessel.control.throttle = 0
		
		#vessel.vessel.auto_pilot.target_direction = self.flight.retrograde
		self.while_below_apogee( [[ self.set_auto_direction , ('retrograde',) ]], 5)
		self.run_experiments()


		#vessel.next_stage()
		self.keep_retrograde()	
	
	def run(self):
		heading = 170
		vessel = self.Vessel

		#vessel.vessel.sas=True
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(90, heading)
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		## 1st 4 engines and main
		self.keep_logging(5,[])
		
		self.dlogt = 0.5
		#vessel.vessel.auto_pilot.target_pitch_and_heading(75, heading)
		self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (80, heading) ]] )
		
		self.keep_logging(2,[])
		
		vessel.next_stage() # decouple 4
		
		self.keep_logging(5,[])
		## big booster
		self.dlogt=2
		#vessel.next_stage() # start big booster (ALREADY RUNNING)
		self.keep_logging(2,[])
		
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (80, heading) ]] , 0.8, 5 )
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (70, heading) ]] , 0.6, 5 )
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (60, heading) ]] , 0.3, 5 )
	#	self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (50, heading) ]] )
		
		#self.while_has_fuel( [[ self.target_min_drag , () ]] )
		self.while_has_fuel( [[ self.set_auto_direction , ('prograde',) ]] )
		vessel.next_stage()	#	decouple
		
		
		## liquid
		self.keep_logging(1,[])
		start_fuel = vessel.vessel.resources.amount('LiquidFuel')
		self.dlogt=10
		vessel.vessel.control.throttle = .1
		vessel.next_stage() # start the engine
		self.while_resource( 'LiquidFuel' , [[ self.set_auto_direction , ('prograde',) ]] , 0.5, 5 )
		vessel.vessel.control.throttle = 0
		
		#vessel.vessel.auto_pilot.target_direction = self.flight.retrograde
		self.while_below_apogee( [[ self.set_auto_direction , ('retrograde',) ]], 5)
		self.run_experiments()


		#vessel.next_stage()
		#self.keep_retrograde()	
		self.keep_logging(1,[[self.set_auto_direction, ('retrograde',)]])


class Higher(Leaper):
			
	def run(self):
		heading = 170
		vessel = self.Vessel

		#vessel.vessel.sas=True
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(90, heading)
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		## 1st 4 engines
		self.keep_logging(5,[])
		
		self.dlogt = 0.5
		#vessel.vessel.auto_pilot.target_pitch_and_heading(75, heading)
		self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (88, heading) ]] )
		
		self.keep_logging(2,[])
		
		vessel.next_stage() # decouple
		
		self.keep_logging(5,[[ self.target_min_drag , () ]])
		## big booster
		self.dlogt=2
		vessel.next_stage() # start big booster
		self.keep_logging(2,[[ self.target_min_drag , () ]])
		
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (80, heading) ]] , 0.8, 5 )
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (70, heading) ]] , 0.6, 5 )
	#	self.while_resource( 'SolidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (60, heading) ]] , 0.3, 5 )
	#	self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (50, heading) ]] )
		self.while_has_fuel( [[ self.target_min_drag , () ]] )
		vessel.next_stage()	#	decouple
		
		
		## liquid
		self.keep_logging(1,[[ self.target_min_drag , () ]])
		start_fuel = vessel.vessel.resources.amount('LiquidFuel')
		self.dlogt=10
		vessel.vessel.control.throttle = .1
		vessel.next_stage() # start the engine
		self.while_resource( 'LiquidFuel' , [[ self.set_auto_direction , ('prograde',) ]] , 0.2, 5 )
		vessel.vessel.control.throttle = 0
		
		#vessel.vessel.auto_pilot.target_direction = self.flight.retrograde
		self.while_below_apogee( [[ self.set_auto_direction , ('retrograde',) ]], 5)
		self.run_experiments()


		#vessel.next_stage()
		self.keep_retrograde()	
		
class Moon(Leaper):## ship - orbiter2
	def run(self):
		heading = 90
		vessel = self.Vessel

		#vessel.vessel.sas=True
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(80, heading)
		vessel.vessel.control.throttle = .6
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		## 1st 2 engines and main
		self.keep_logging(5,[])
		
		self.dlogt = 0.5
		#vessel.vessel.auto_pilot.target_pitch_and_heading(75, heading)
		self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (65, heading) ]] )
		
		self.keep_logging(2,[])
		
		vessel.next_stage() # decouple 4
		
		self.keep_logging(5,[])
		## big booster
		self.dlogt=2
		#vessel.next_stage() # start big booster (ALREADY RUNNING)
		self.keep_logging(2,[])
		
		start_fuel = vessel.vessel.resources.amount('LiquidFuel')
		self.dlogt=5
		vessel.vessel.control.throttle = .5
		
		self.while_resource( 'LiquidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (45, heading) ]] , 0.8, 5 )
		self.while_resource( 'LiquidFuel' , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (25, heading) ]] , 0.1, 5 )
		vessel.vessel.control.throttle = .1
		self.while_has_fuel( [[ self.set_auto_direction , ('prograde',) ]] , 5)
		vessel.next_stage()	#	decouple
		vessel.vessel.control.throttle = 0
		

		
		#vessel.vessel.auto_pilot.target_direction = self.flight.retrograde
		self.while_below_apogee( [[ self.set_auto_direction , ('prograde',) ]], 5)
		
		
		
		
		### land
		self.while_has_fuel( [[ self.set_auto_direction , ('retrograde',) ]] )
		
		vessel.next_stage()
		vessel.next_stage()#chute
		self.keep_logging(1,[[self.set_auto_direction, ('retrograde',)]])

		
class Satellite(Leaper):## ship - orbiter-sat
	def run(self):
		heading = 90
		vessel = self.Vessel

		#vessel.vessel.sas=True
		
		vessel.vessel.auto_pilot.engage()
		#vessel.vessel.auto_pilot.overshoot = (.001,.001,.01)
		vessel.vessel.auto_pilot.target_pitch_and_heading(88, heading)
		vessel.vessel.control.throttle = 0.5
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		
		## 1st 2 engines and main
		self.keep_logging(5,[])
		
		self.dlogt = 0.5
		#vessel.vessel.auto_pilot.target_pitch_and_heading(75, heading)
		
		
		vessel.vessel.auto_pilot.target_pitch_and_heading(86, heading)
		self.keep_logging( 5, [])
		vessel.vessel.auto_pilot.target_pitch_and_heading(84, heading)
		self.keep_logging( 10, [])
		vessel.vessel.auto_pilot.target_pitch_and_heading(82, heading)
		self.keep_logging( 10, [])
		vessel.vessel.auto_pilot.target_pitch_and_heading(80, heading)
		self.keep_logging( 10, [])
		self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (78, heading) ]] )
		#self.while_has_fuel( [[ self.set_auto_direction , ('prograde',) ]] )
		
		#self.keep_logging(1,[])
		
		self.keep_logging(1,[])
		vessel.vessel.control.throttle = 1
		self.keep_logging(1,[])
		vessel.next_stage() # decouple 4
		self.keep_logging(2,[])
		
		vessel.vessel.auto_pilot.target_pitch_and_heading(65, heading)
		vessel.vessel.control.throttle = 0.01
		self.keep_logging(1,[])
		
		cond = lambda : vessel.vessel.auto_pilot.error > 25
		self.while_general( cond , [])
				
		## raise apoapsis
		vessel.vessel.control.throttle = .5
		self.keep_logging(2,[])
		cond = lambda : self.orbit.apoapsis_altitude < 80000
		self.while_general( cond , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (35, heading) ]] , 1)
		vessel.vessel.control.throttle = 0
		
		##raise periapsis
		vessel.vessel.control.throttle = .2
		cond = lambda : self.orbit.periapsis_altitude < 80000
		#self.while_general( cond , [[ self.set_auto_direction , ('prograde',) ]] , 1)
		self.while_general( cond , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (5, heading) ]] , 1)
		vessel.vessel.control.throttle = 0
		
		
		
		#~ ### land
		#~ vessel.vessel.control.throttle = 0.1
		#~ self.while_has_fuel( [[ self.set_auto_direction , ('normal',) ]] )
		
		#~ vessel.next_stage()
		
		#~ self.keep_logging(10,[[self.set_auto_direction , ('radial',) ]])
		#~ self.vessel.auto_pilot.disengage()
		
class Plane(Leaper):
	def run(self):
		self.flight = self.vessel.flight(self.vessel.surface_reference_frame)
		
		heading_def = 90#def
		heading = 150
		vessel = self.Vessel
		#vessel.vessel.control.input_mode = CC.sc.ControlInputMode(1)
		
		vessel.vessel.auto_pilot.engage()
		#vessel.vessel.auto_pilot.target_pitch_and_heading(-5, heading_def)
		#vessel.vessel.auto_pilot.target_heading = heading_def
		vessel.vessel.control.throttle = 1
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		#vessel.vessel.control.pitch = 0.1
		self.keep_logging(5, [])
		
		
		
		
		cond = lambda : self.flight.speed < 100
		self.while_general( cond , [[ self.set_auto_direction , ('prograde',) ]] )
		
		
		
		## take-off
		
		cond = lambda : self.flight.pitch < 10
		vessel.vessel.control.pitch = 1
		self.while_general( cond, [] , 0.02)
		
		
		## take-off
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch = 15
		vessel.vessel.auto_pilot.target_heading = heading_def
		self.keep_logging(15, [] )
		
		vessel.vessel.auto_pilot.target_pitch = 45
		vessel.vessel.auto_pilot.target_heading = heading
		
		cond = lambda : self.flight.elevation < 6000
		self.while_general( cond, [] )
		
		vessel.vessel.auto_pilot.target_pitch_and_heading(5,heading)
		
class LongPlane(Leaper):
	def run(self):
		self.flight = self.vessel.flight(self.vessel.surface_reference_frame)
		
		heading_def = 90#def
		heading = 150
		vessel = self.Vessel
		vessel.vessel.auto_pilot.engage()
		vessel.vessel.auto_pilot.target_pitch_and_heading(90, heading_def)
		vessel.vessel.control.throttle = 1
		
		self.readData(self.data)
		vessel.next_stage()
		print('Launch!')
		self.keep_logging(5, [])
		self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (85, heading_def) ]] )
		
		vessel.next_stage()
		
		## raise apoapsis
		vessel.vessel.control.throttle = 1
		cond = lambda : self.orbit.apoapsis_altitude < 15000
		self.while_general( cond , [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (65, heading_def) ]] , 1)
		
		self.while_has_fuel( [[ vessel.vessel.auto_pilot.target_pitch_and_heading , (5, heading) ]] )
		vessel.next_stage()
		
		self.while_below_apogee( [], 2)
		
		vessel.next_stage()
		
		self.keep_logging( 100, [] )

#miss = Leaper(CC.sc.active_vessel)
#miss = Higher(CC.sc.active_vessel)
#miss = Moon(CC.sc.active_vessel)
miss = Satellite(CC.sc.active_vessel)
#miss = Plane(CC.sc.active_vessel)
#miss = LongPlane(CC.sc.active_vessel)




class Sim:
	def __init__(self, m0, m1, dm, T):
		self.m = m0
		self.mlim = m1
		self.dm = dm
		self.T = T
		
	def sim(self, AC, ts, denss):
		tprev = 0
		
		T = self.T
		M = self.m
		
		h = 0
		v = 0
		
		out = []
		for i in range(len(ts)):
			dt = ts[i] - tprev
			
			if M < self.mlim : T = 0
			
			F = T - AC*v*denss[i]
			a = F/M - 9.81
			
			dv = a*dt
			dh = v*dt
			
			h += dh
			v += dv
			
			if T > 0: M -= self.dm*dt
			
			out += [h]
			
			tprev = ts[i]
		
		return out



def stopData():
	dt = miss.data['t'].copy()
	dh = miss.data['alt'].copy()
	dd = miss.data['dens'].copy()
	
	plt.figure()
	plt.plot(dt,dh)
	plt.show(0)
	
	
	#~ testA = np.linspace(1,150,300)
	#~ valsA = np.zeros_like(testA)
	#~ valsRef = np.array(dh)
	
	#~ s = Sim(3500,1800,118,197000)
	
	#~ for i in range(len(testA)):
		#~ pred = s.sim(testA[i],dt,dd)
		#~ valsA[i] = np.average(np.abs(pred-valsRef))
	
	#~ plt.figure()
	#~ plt.plot(testA, valsA)
	#~ plt.show(0)

	#~ plt.figure()
	#~ pred = s.sim(65,dt,dd)
	#~ plt.plot(dt,pred)
	#~ plt.show(0)

	#~ return testA, valsA
