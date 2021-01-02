from bodysim2d import *

class OrbitSim:
	def __init__(self, Rocket):
		self.rocket = rocket
		self.kerbin = Ball(co([0,0]), co([0,0]), 5.3e22, 6e5)
		self.bodies = Bodies([kerbin, rocket])

		#~ booster_l = Engine( [250e3,300e3], [175,210], 1.6e3, 6.15e3, multi = 4 )
		#~ liquid = Engine( [168e3, 215e3], [250, 320], 3e3, 10e3 )
		#~ terrier = Engine( [14.7e3,60e3], [85,345], 0.5e3, 1e3 )

		#~ booster_l.next_stage = [liquid]
		#~ liquid.next_stage = [terrier]

		#~ #booster_s.activate()
		#~ booster_l.activate()
		#~ liquid.activate()

		#~ rocket.M_payload = 2e3#3e3
		#~ rocket.engines = [booster_l, liquid, terrier]
		#~ rocket.set_target_body(kerbin)
		#~ rocket.dir_change = 3

		atmo = Atmosphere()
		kerbin.atmo = atmo
		kerbin.fixed = True


		print('mass rocket: '+ str(rocket.M))
		print('thrust: '+ str(rocket.total_thrust(atmo.h0)))
		for e in rocket.engines:
			print('engine '+e.name + ' : '+str(e.thrust()))

		self.cap = capturePos(bod.balls)
		self.dt = .1


	def sim(self, T = 500):
		nT = int(T/self.dt)
		for i in range(nT):
			self.bod.iterate(dt)
			self.cap.capture()
		
		









if __name__ == '__main__':

	kerbin = Ball(co([0,0]), co([0,0]), 5.3e22, 6e5)
	rocket = Rocket(co([ 6e5 , 0 ]), co([ 0, 175 ]))

	bod = Bodies([kerbin, rocket])

	#~ booster_s = Engine( [199e3,227e3], [170, 195], 0.8e3, 2.85e3, multi = 2 )
	#~ booster_l = Engine( [250e3,300e3], [175,210], 1.6e3, 6.15e3 )
	#~ liquid = Engine( [168e3, 215e3], [250, 320], 2.3e3, 6e3 )
	#~ terrier = Engine( [14.7e3,60e3], [85,345], 0, 2.8e3 )

	#~ booster_l.next_stage = [liquid]
	#~ liquid.next_stage = [terrier]

	#~ booster_s.activate()
	#~ booster_l.activate()
	#~ #liquid.activate()

	booster_l = Thruster( [250e3,300e3], [175,210], 1.6e3, 6.15e3, multi = 4 )
	liquid = Thruster( [168e3, 215e3], [250, 320], 3e3, 10e3 )
	terrier = Thruster( [14.7e3,60e3], [85,345], 0.5e3, 1e3 )

	booster_l.next_stage = [liquid]
	liquid.next_stage = [terrier]

	#booster_s.activate()
	booster_l.activate()
	liquid.activate()

	rocket.M_payload = 2e3#3e3
	rocket.engines = [booster_l, liquid, terrier]
	rocket.set_target_body(kerbin)
	rocket.dir_change = 3

	atmo = Atmosphere()
	kerbin.atmo = atmo
	kerbin.fixed = True


	#rocket.direction = np.pi/2

	print('mass rocket: '+ str(rocket.M))
	print('thrust: '+ str(rocket.total_thrust(6e5)))
	for e in rocket.engines:
		print(e.thrust())

	cap = capturePos(bod.balls)
	dt = .5
	T = 500
	nT = int(T/dt)
	for i in range(nT):
		bod.iterate(dt)
		cap.capture()
			
			
			
			
	tx = np.array([x for x in cap.tx[rocket.id]])
	ty = np.array([x for x in cap.ty[rocket.id]])
	tnorm = 2e3/np.sqrt( tx[0]*tx[0] + ty[0]*ty[0] )	

	tx *= tnorm
	ty *= tnorm

			
	plt.figure()
	#plt.plot( cap.x[ kerbin.id ] , cap.y[ kerbin.id ] )
	plt.plot( cap.x[ rocket.id ] , cap.y[ rocket.id ] )

	kerbit = plt.Circle((0,0),6e5, edgecolor='red',linewidth='2.2',fill=False)
	plt.gca().add_patch(kerbit)
	plt.scatter([cap.x[ rocket.id ][0],cap.x[ rocket.id ][-1]],[cap.y[ rocket.id ][0],cap.y[ rocket.id ][-1]], color = 'red')

	for i in range(0,cap.n+1):
		plt.plot([cap.x[rocket.id][i], cap.x[rocket.id][i] + tx[i]],[cap.y[rocket.id][i], cap.y[rocket.id][i] + ty[i]], color = 'black', alpha=0.6)

	plt.show(0)



	plt.figure()
	radius = []
	for i in range(cap.n+1):
		dr = np.array( [cap.x[rocket.id][i] - cap.x[kerbin.id][i] , cap.y[rocket.id][i] - cap.y[kerbin.id][i]] )
		radius += [ np.linalg.norm(dr) - kerbin.R ]

	plt.plot( np.linspace(0, T, nT+1), radius )
	plt.show(0)


	plt.figure()
	plt.plot( np.linspace(0, T, nT+1), cap.direction[rocket.id] )
	plt.plot( np.linspace(0, T, nT+1), [x / cap.thrust[rocket.id][0] for x in cap.thrust[rocket.id] ])
	plt.plot( np.linspace(0, T, nT+1), [x / cap.mass[rocket.id][0] for x in cap.mass[rocket.id] ])
	plt.show(0)
	plt.figure()
	plt.plot( np.linspace(0, T, nT+1), cap.mass[rocket.id] )
	plt.show(0)
