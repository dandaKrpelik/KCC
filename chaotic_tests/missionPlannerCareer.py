import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir+'/KCC') 
sys.path.insert(0,parentdir+'/odyn') 

from Missions import *
import math
import Sim
import debug


		
class TakeOffPhase(SingleVesselPhase):
	##fix?
	
	def init(self,miss):
		Vessel = miss.vessels[ self.ves_name ]
		v = Vessel.vessel
		
		p = AutoPilot(Vessel)
		p.target_head_ele(self.heading , self.ele )
		p.throttle(self.thrust)
		
		miss.update_pilot(p)
		print('Launch!!!')
		Vessel.next_stage()

class BurnFuel(SingleVesselPhase):
	##fix?
	decouple_lag = 1
	stage = True
	
		
	def condition(self,miss):
		Vessel = miss.vessels[ self.ves_name ]
		return not Vessel.engines_have_fuel(Vessel.stage-1)
		
		
	def init(self,miss):
		Vessel = miss.vessels[ self.ves_name ]
		
		p = miss.pilots[Vessel.name]
		p.target_head_ele(self.heading, self.ele)
		p.throttle(self.thrust)
		
		print('burning all fuel in stage')
		
	def end(self,miss):
		if self.stage:
			Vessel = miss.vessels[ self.ves_name ]
			
			p = miss.pilots[Vessel.name]
			p.throttle(0)
			time.sleep( self.decouple_lag )
			Vessel.next_stage()
			time.sleep( self.decouple_lag )
			p.throttle(self.thrust)

class ReachApoapsis(SingleVesselPhase):
	flight = None
	stage = True
	
	def condition(self, miss):
		Vessel = miss.vessels[ self.ves_name ]
		return self.flight.vertical_speed < 0 or Vessel.vessel.orbit.time_to_apoapsis < self.dt
	
	def __init__(self, name, dt = 0):
		SingleVesselPhase.__init__(self,name)
		self.dt = dt
		
	def init(self, miss):
		Vessel = miss.vessels[ self.ves_name ]
		v = Vessel.vessel
		
		self.flight = v.flight(v.orbit.body.reference_frame)
		
		p = miss.pilots[Vessel.name]
		p.target_roll(0)
		#p.set_reference_frame(v.orbital_reference_frame)
		#p.target_head_ele(self.heading, self.ele)
		p.target_orbital_dir('prograde')

	def end(self, miss):
		if self.stage:
			Vessel = miss.vessels[ self.ves_name ]
			Vessel.next_stage()

class RaiseOrbit(SingleVesselPhase):
	sign = 1
	target_dir = 'prograde'
	roll = 0
	
	def __init__(self, name, tag = 'apoapsis', height = 80000):
		SingleVesselPhase.__init__(self,name)
		
		assert tag in ['apoapsis','periapsis']
		self.tag = tag
		self.height = height
		
		if height < 0 :
			#self.height =  -height
			self.sign = -1
			self.target_dir = 'retrograde'
		
	def condition(self, miss):
		orbit = miss.vessels[ Vessel.name ].vessel.orbit
		return self.sign * orbit.__getattribute__(self.tag+'_altitude') > self.height
		
	def init(self, miss):
		Vessel = miss.vessels[ self.ves_name ]
		v = Vessel.vessel
		
		p = miss.pilots[Vessel.name]
		p.engage()
		if not self.target_dir is None:
			p.update_reference_frame(v.orbital_reference_frame)
			p.target_orbital_dir( self.target_dir )
		
		p.throttle(self.thrust)
		
	def end(self,miss):
		Vessel = miss.vessels[ self.ves_name ]
		v = Vessel.vessel
		p = miss.pilots[Vessel.name]
		p.throttle(0)

class DescentOnLevelPhase(SingleVesselPhase):
	flight = None
	height = 0
	roll = 0
	
	def condition(self, miss):
		flight = miss.vessels[self.ves_name].vessel.flight( miss.pilots[self.ves_name].ap )
		return flight.surface_altitude <= self.height

	def init(self,miss):
		Vessel = miss.vessels[ self.ves_name ]
		v = Vessel.vessel
		
		p = miss.pilots[Vessel.name]
		p.update_reference_frame(v.orbital_reference_frame)
		p.target_orbital_dir('retrograde')
		
		p.throttle(self.thrust)		
		

class FinalDescentPhase(DescentOnLevelPhase):
	roll = 0
	
	## add condition
	def init(self, miss):
		DescentOnLevelPhase.init(self,miss)
		
		Vessel = miss.vessels[ self.ves_name ]
		v = Vessel.vessel
		
		p = miss.pilots[Vessel.name]
		
		p.update_reference_frame(v.surface_reference_frame)
		p.target_orbital_dir('retrograde')
		
		p.throttle(self.thrust)		
	
## airplane	
class aTakeOffPhase(SingleVesselPhase):
	##fix?
	dt = 10
	
	def condition(self,miss):
		### velocity -> lift
		
		ts = miss.data_log['t']
		return ts[-1] - ts[0] > self.dt
		
	def init(self,miss):
		Vessel = miss.vessels[ self.ves_name ]
		v = Vessel.vessel
		
		p = miss.pilots[ self.ves_name]
		p.throttle(self.thrust)
		
		p.deactivate()
		v.sas = True
		
		print('Launch!!!') 
		Vessel.next_stage()		
	

	
def getLeapMission( target = '' ):
	## Arrow-SH-1
	print('mission - target '+str(target))
	
	miss = Mission()
	miss.add_vessel( Vessel(CC.sc.active_vessel) )
	
	v = miss.vessels['main'].vessel
	v.auto_pilot.reference_frame = v.surface_reference_frame
	
	
	
	heading = 180
	p1 = TakeOffPhase('take off'); p1.heading = heading
	
	p1.heading = heading
	p1.sas = True
	p1.ele = 85
	p1.thrust = 1
	#p1.dt = 30###
	miss.add_phase( p1 )
	
	p2 = BurnFuel('burn boosters')
	p2.sas=True
	p2.heading = heading
	p2.ele = 80
	p2.thrust = 1
	miss.add_phase( p2 )
	
	##replace pilot
	new_pilot = AutoPilot(miss.vessels['main'])
	
	for w in CC.sc.waypoint_manager.waypoints:
		if w.name == target:
			new_pilot.target = ( w.latitude, w.longitude )
			new_pilot.track = True
			print('target found!')
	
	
	class AdjustPilot(SingleVesselPhase):
		def init(self, miss):
			miss.update_pilot(new_pilot)
			new_pilot.engage()
			new_pilot.target_elevation(65)

	c1 = AdjustPilot('adjust direction')
	c1.thrust = 1
	miss.add_phase(c1)
	
	
	
	p3 = BurnFuel('burn liquid')
	p3.ele = 25
	p3.heading = heading
	p3.thrust = 0.8
	p3.stage = False
	miss.add_phase( p3 )
	
	
	
	
	p4 = ReachApoapsis('wait for apogee')
	p4.heading = heading
	p4.ele = 15
	p4.stage = False
	miss.add_phase( p4 )

	#~ p4 = RaiseOrbit('reaching testing height', height = 105000)
	#~ p4.target_dir = None
	#~ p4.thrust = 1
	#~ miss.add_phase(p4)
	
	#~ pp = ReachApoapsis('awit for apo')
	#~ miss.add_phase(pp)
	
	p5 = BurnFuel('glide')
	p5.ele = 15
	p5.heading = heading
	p5.thrust = 0.3
	miss.add_phase(p5)
	
	
	return miss
	
def getLongLeapMission():
	## orbiter-tourist-s
	miss = Mission()
	miss.add_vessel( Vessel(CC.sc.active_vessel) )
	
	v = miss.vessels['main'].vessel
	v.auto_pilot.reference_frame = v.surface_reference_frame
	
	heading = 90
	p1 = TakeOffPhase('take off'); p1.heading = heading
	p1.heading = heading
	p1.ele = 85
	p1.thrust = 1
	miss.add_phase( p1 )
	
	p2 = BurnFuel('burn boosters')
	p2.heading = heading
	p2.ele = 88
	p2.thrust = 0.6
	miss.add_phase( p2 )
	
	class AdjustPilot(SingleVesselPhase):
		def init(self, miss):
			p = miss.pilots['main']
			p.target_roll = 0
			p.target_head_ele(heading, 80)
			p.throttle(self.thrust)
			p.engage()

	c1 = AdjustPilot('adjust direction')
	c1.thrust = 0.2
	miss.add_phase(c1)

	c2 = SingleVesselPhase('wait')
	c2.dt = 10
	c2.ele = 75
	miss.add_phase(c2)
	
	p3 = RaiseOrbit('raise apogee', 'apoapsis', 100000)
	p3.thrust = 1
	miss.add_phase( p3 )
	
	p4 = ReachApoapsis('wait for apogee', dt = 10)
	p4.stage = False
	miss.add_phase( p4 )
	
	return miss	

def getOrbitMission(height = 150e3):
	## orbiter-tourist-s
	## orbiter-base
	
	## orbiter - S
	miss = Mission()
	miss.add_vessel( Vessel(CC.sc.active_vessel) )
	
	v = miss.vessels['main'].vessel
	v.auto_pilot.reference_frame = v.surface_reference_frame
	
	heading = 340
	roll = 0
	
	p1 = TakeOffPhase('take off')
	p1.heading = heading
	p1.ele = 90
	p1.thrust = 1
	miss.add_phase( p1 )
	
	p2 = BurnFuel('burn boosters')
	p2.heading = heading
	p2.ele = 85
	p2.thrust = 1
	miss.add_phase( p2 )
	
	c2 = SingleVesselPhase('wait')
	c2.sas = True
	c2.dt = 25
	c2.thrust = 1
	miss.add_phase(c2)
	
	p3 = RaiseOrbit('raise apogee', 'apoapsis', height)
	#p3.sas = True
	p3.thrust = 1
	miss.add_phase( p3 )
	
	dt = 35
	p4 = ReachApoapsis('wait for apogee', dt = dt)
	p4.stage = False
	miss.add_phase( p4 )
	
	p5 = RaiseOrbit('raise periapsis', 'periapsis', height)
	p5.thrust = 1
	miss.add_phase(p5)
	

	return miss
	
def getLandMission():
	## orbiter-tourist-s
	miss = Mission()
	miss.add_vessel( Vessel(CC.sc.active_vessel) )
	
	v = miss.vessels['main'].vessel
	v.auto_pilot.reference_frame = v.orbital_reference_frame
	
	p0 = RaiseOrbit('slow down','periapsis', -500)
	p0.thrust = 0.1
	miss.add_phase(p0)
	
	p1 = DescentOnLevelPhase('descent-1')
	p1.height = 10000
	p1.thrust = 0
	miss.add_phase( p1 )
	
	
	p2 = FinalDescentPhase('descent-x')
	p2.thrust = 0.1
	miss.add_phase( p2 )
		
	return miss
	
def getLongPlaneMission(destination = None):
	## muchacho-L-1
	miss = Mission()
	miss.add_vessel( Vessel(CC.sc.active_vessel) )
	
	heading = 90
	p1 = TakeOffPhase('take off'); p1.heading = heading
	p1.heading = heading
	p1.ele = 85
	p1.thrust = 1
	p1.dt=5
	miss.add_phase( p1 )
	
	p2 = BurnFuel('burn boosters')
	p2.heading = heading
	p2.ele = 80
	p2.thrust = 0.8
	miss.add_phase( p2 )	

	#~ class AdjustPilot(SingleVesselPhase):
		#~ def init(self, miss):
			#~ p = miss.pilots['main']
			#~ p.target_roll = 0
			#~ p.target_head_ele(heading, 80)
			#~ p.throttle(self.thrust)
			#~ p.engage()

	#~ c1 = AdjustPilot('adjust direction')
	#~ c1.thrust = 0.2
	#~ miss.add_phase(c1)

	#~ c2 = SingleVesselPhase('wait')
	#~ c2.dt = 10
	#~ c2.ele = 70
	#~ miss.add_phase(c2)

	p3 = BurnFuel('burn liquid')
	p3.ele = 55
	p3.heading = heading
	p3.thrust = 0.8
	miss.add_phase( p3 )
	
	p4 = ReachApoapsis('wait for apogee')
	p4.heading = heading
	p4.ele = 15
	miss.add_phase( p4 )	


	return miss
	
class DataMission(Mission):
	log_keys = ['t', 'alt', 'alt_o',
			'atmo_dens', 'velocity', 'aoa',
			'drag', 'drag_coef', 'aero',
			'aero_coef', 'mass', 'thrust', 'isp' ,
			'o_speed', 'o_dir', 's_dir', 's_head']
	def log_data(self):
		data = self.data_log
		v = self.vessels['main']
		
		data['alt_o'] += [v.altitude]
		data['alt'] += [v.altitude - data['alt_o'][0] ]
		
		data['drag'] += [np.linalg.norm(v.drag)]
		data['drag_coef'] += [v.drag_coef]
		
		data['aero'] += [np.linalg.norm(v.aero)]
		data['aero_coef'] += [v.aero_coef]
		
		data['atmo_dens'] += [v.atmo_dens]
		data['velocity'] += [v.velocity]		
		data['aoa'] += [ v.aoa ]
		
		data['mass'] += [v.mass]
		data['thrust'] += [v.thrust]
		data['isp'] += [v.isp]
		
		data['o_speed'] += [v.orbital_speed]
		data['o_dir'] += [v.orbit_heading]
		data['s_dir'] += [v.surface_dir]
		data['s_head'] += [v.surface_heading]
	
def getObserveFlightMission(orbit_height = 150e3, launch_heading=90):
	miss = DataMission()
	miss.add_vessel( Vessel(CC.sc.active_vessel) )
	
	heading = launch_heading
	height = orbit_height
	
	p1 = TakeOffPhase('take off');
	p1.heading = heading
	p1.ele = 80
	p1.thrust = 1
	#p1.dt=5
	miss.add_phase( p1 )
	
	p2 = BurnFuel('burn boosters')
	p2.heading = heading
	p2.ele = 70
	p2.thrust = 1
	miss.add_phase( p2 )	


	c2 = SingleVesselPhase('wait')
	c2.sas = True
	c2.dt=15
	c2.thrust = 1
	miss.add_phase(c2)

	miss.addq_change_update_time(1)
	
	p3 = RaiseOrbit('raise apogee', 'apoapsis', height)
	#p3.sas = True
	p3.thrust = 1
	#p3.ele = 65
	miss.add_phase( p3 )
	
	miss.addq_change_update_time(5)
		
	dt = 35
	p4 = ReachApoapsis('wait for apogee', dt = dt)
	p4.thrust = 0
	p4.stage = False
	miss.add_phase( p4 )
	
	miss.addq_change_update_time(1.5)
	
	p5 = RaiseOrbit('raise periapsis', 'periapsis', height)
	p5.thrust = 1
	miss.add_phase(p5)
	
	miss.addq_stop_updates()
	
	return miss
	
if __name__ == '__main__':
		
	#~ miss = getLeapMission()

	#~ miss.phases['burn boosters'].stage = False
	#~ miss.phases['burn liquid'].stage = False

	#~ #orbit and land
	#~ miss = getOrbitMission(height = 250e3)
	#~ land = getLandMission()
	#~ for p in land.phases:
		#~ miss.add_phase( land.phases[p] )
		
	#~ destination = None
	#~ #miss = getLongPlaneMission(destination)
	#~ miss = getLongLeapMission()


	#miss = getLeapMission( target = 'Zone 27-L3' )

	miss = getObserveFlightMission()
	#~ land = getLandMission()
	#~ for p in land.phases:
		#~ miss.add_phase( land.phases[p] )


def show_data_log(miss,tag):
	plt.figure()
	plt.plot(miss.data_log['t'], miss.data_log[tag])
	plt.ylabel = tag
	plt.xlabel = 'time(s)'
	
	plt.show(0)


def showDataDisplay(miss, offset = 10):
	tags1 = ['alt', 'atmo_dens', 'velocity', 'aoa', 'drag_coef',  'aero_coef', 'mass', 'thrust', 'isp' ]
	tags2 = ['alt', 'atmo_dens', 'velocity',  'aero', 'mass', 'thrust', 'isp' ]
	tags3 = ['alt', 'o_speed', 'o_dir', 's_dir', 's_head' ]
	
	tgs = [tags1, tags2, tags3]
	nts = [len(t) for t in tgs]
	figs = []
	
	for tags in tgs:
		figs += [plt.figure()]
		
	while(True):
		t = miss.data_log['t']
		for i in range(len(tgs)):
			tags = tgs[i]
			nt = nts[i]
			fig = figs[i]
			
			fig.clf()
			
		
			cnt = 0
			for tag in tags:
				cnt += 1
				spl = '%d%d%d' % (nt,1,cnt)
				ax = fig.add_subplot(spl)
			
			
				N = min(len(t),len(miss.data_log[tag]))
				if N < offset: continue
			
			
				ax.plot( t[offset:N] , miss.data_log[tag][offset:N])
				ax.set_ylabel(tag)
			
		plt.xlabel = 'time(s)'
		plt.show(block = False)
		
		plt.pause(1)
	
def endAndLog(miss, fname = 'default'):
	miss.disengage()
	import csv
	
	tags = ['alt', 'atmo_dens', 'velocity', 'aoa', 'drag', 'drag_coef', 'aero', 'aero_coef', 'mass', 'thrust', 'isp', 'o_speed' ]
	tagsv = ['o_dir', 's_dir', 's_head']
	nt = len(tags)
	
	Ns = [len(miss.data_log['t'])]
	Ns += [ len(miss.data_log[tag]) for tag in tags ] 
	N = min(Ns)
	
	with open('data/'+fname+'.csv','w') as f:
		w = csv.writer(f)
		names = ['t'] + tags
		for vt in tagsv:
			names += ['%s_%d' % (vt,i+1) for i in range(3)]
			
		w.writerow(names)
		for i in range(N):
			row = [miss.data_log['t'][i]]
			row += [ miss.data_log[tag][i] for tag in tags ]
			for vt in tagsv:
				x = miss.data_log[vt][i]
				row += x
			w.writerow(row)
	
