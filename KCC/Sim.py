import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir+'/odyn') 

from Missions import *
from orbitPlanner import *
import math


def getCurrentSetUp(miss, ves_name = 'main'):
	Ves = miss.vessels[ves_name]
	orbit = Ves.vessel.orbit
	body = orbit.body
	
	Planet = Ball(co([0,0]),co([0,0]),body.mass, body.equatorial_radius)
	Planet.fixed = True
	atmo = Atmosphere()
	Planet.atmo = atmo
	
	rocket = Rocket( co([orbit.apoapsis,0]), co([0,orbit.speed])  , Planet)

	bod = Bodies([Planet, rocket])

	rocket_engines = []

	max_stage = -1
	for stage in Ves.engines:
		if stage > max_stage: max_stage = stage
		
		total_LO_mass = 0
		dry_mass = 0
		N_LOengines = 0
		
		for p in Ves.vessel.parts.in_decouple_stage(stage):
			dry_mass += p.dry_mass
			if 'LiquidFuel' in p.resources.names:
				total_LO_mass += p.mass - p.dry_mass
			if p.engine and 'LiquidFuel' in p.engine.propellant_names:
				N_LOengines += 1
		
		sittingDuck = Thruster(0,0,dry_mass, 0)
		rocket_engines += [sittingDuck]
		
		for e in Ves.engines[stage]:
			
			dry_mass -= e.engine.part.dry_mass		## accounted separatedly --- > inefficiently
			
			extra_fuel = 0
			if 'LiquidFuel' in e.engine.propellant_names:	## assume symetrical distribution to this stage engines
				extra_fuel += total_LO_mass / N_LOengines
				
			th = Thruster( [ e.engine.max_thrust , e.engine.max_vacuum_thrust ] , 
				[e.engine.kerbin_sea_level_specific_impulse, e.engine.vacuum_specific_impulse ] ,
				e.engine.part.dry_mass, e.engine.part.mass - e.engine.part.dry_mass + extra_fuel)
			th.observers += [ sittingDuck ]		## to kill it when decouple
				
			rocket_engines += [th]
			e.engine._p_thruster = th
			
	for e in Ves.vessel.parts.engines:
		if e.part.stage == max_stage:
			e._p_thruster.activate()
			
			
			### staging
	sts = list(Ves.engines.keys())
	sts.sort(key = lambda x:-x)
	for i in range(1,len(sts)):
		for e in Ves.engines[ sts[i] ]:
			for eprev in Ves.engines[ sts[i-1] ]:
				eprev.engine._p_thruster.next_stage += [ e.engine._p_thruster ]
		
		
	rocket.engines = rocket_engines
	rocket.dir_change = 3
	
	print('mass rocket: '+ str(rocket.M))
	print('thrust: '+ str(rocket.total_thrust(atmo.h0)))
	for e in rocket.engines:
		print(e.thrust())

	return bod

def simandplot(miss):
	bod = Sim.getCurrentSetUp(miss)
	cap = Sim.capturePos(bod.balls)
	
	
	
	dt = .5
	T = 500
	nT = int(T/dt)
	for i in range(nT):
		bod.iterate(dt)
		cap.capture()

	planet = bod.balls[0]
	rocket = bod.balls[1]


	tx = np.array([x for x in cap.tx[rocket.id]])
	ty = np.array([x for x in cap.ty[rocket.id]])
	tnorm = 2e3/np.sqrt( tx[0]*tx[0] + ty[0]*ty[0] )	

	tx *= tnorm
	ty *= tnorm

	plt.figure()
	plt.plot( cap.x[ rocket.id ] , cap.y[ rocket.id ] )

	kerbit = plt.Circle((0,0),planet.R, edgecolor='red',linewidth='2.2',fill=False)
	plt.gca().add_patch(kerbit)
	plt.scatter([cap.x[ rocket.id ][0],cap.x[ rocket.id ][-1]],[cap.y[ rocket.id ][0],cap.y[ rocket.id ][-1]], color = 'red')

	for i in range(0,cap.n+1):
		plt.plot([cap.x[rocket.id][i], cap.x[rocket.id][i] + tx[i]],[cap.y[rocket.id][i], cap.y[rocket.id][i] + ty[i]], color = 'black', alpha=0.6)

	plt.show(0)



	plt.figure()
	radius = []
	for i in range(cap.n+1):
		dr = np.array( [cap.x[rocket.id][i] - cap.x[planet.id][i] , cap.y[rocket.id][i] - cap.y[planet.id][i]] )
		radius += [ np.linalg.norm(dr) - planet.R ]

	plt.plot( np.linspace(0, T, nT+1), radius )
	plt.show(0)


	plt.figure()
	plt.plot( np.linspace(0, T, nT+1), cap.direction[rocket.id] )
	plt.plot( np.linspace(0, T, nT+1), [x / cap.thrust[rocket.id][0] if cap.thrust[rocket.id][0] > 0 else 0 for x in cap.thrust[rocket.id] ])
	plt.plot( np.linspace(0, T, nT+1), [x / cap.mass[rocket.id][0] for x in cap.mass[rocket.id] ])
	plt.show(0)
	plt.figure()
	plt.plot( np.linspace(0, T, nT+1), cap.mass[rocket.id] )
	plt.show(0)
