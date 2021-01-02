import numpy as np
import sys
sys.path.insert(1, '../odyn')
import bodysim3d as bs

def dkcc_mun_attribs(miss):
	v = miss.vessels['main']
	
	ker = v.vessel.orbit.body
	mun = ker.satellites[0]
	rf = ker.non_rotating_reference_frame
	
	o = mun.orbit
	
	R = o.apoapsis
	mu = ker.gravitational_parameter
	T = 2*np.pi*np.sqrt( R**3 / mu )
	omega = 2*np.pi / T
	v = omega * R
	
	print(R)
	print(mu)
	print( 'M * G = %f' % ( ker.mass * 6.67e-11 ) )
	print(T)
	print(omega)
	print(v)
	
	
	x = np.array( mun.position(rf) )
	dx = np.array( mun.velocity(rf) )
	r = np.linalg.norm(x)
	
	aNewton = -x * mu * np.power( r, -3 )
	aCentri = -x * omega**2 
	
	print(x)
	print(dx)
	print(r)
	print(aNewton)
	print(aCentri)
	
	
	
	
	return ker, mun
	
def dstat_mun_dyn(cap):
	bods = cap.bodies
	
	T = 141115.4
	omega = 2*np.pi / T
	
	
	dt = 15.
	
	for i in range(5):
		
		for b in bods:
			if b.fixed: continue
			dv = np.zeros(3)
			for c in bods:
				if b.id == c.id : continue
					
				dist = b.d2(c)
				a =   b.to(c)  * ( 6.67e-11 * c.M / dist)
				dv += a
				
				print('-------- N = %d --------' % i )
				print(b.v.x)
				print(dist)
				print(a)
				print(dv)
				
				print('mu = %f' % (6.67e-11 * c.M))
				print(np.linalg.norm(a))
				print(np.sqrt(dist) * omega**2)
				
			b.x.x += b.v.x * dt + 0.5 * dv * dt ** 2
			b.v.x += dv * dt
	
	ker,mun = bods[0],bods[1]
	return ker, mun
	
	
def dstat_rocket_dyn(cap, eles = None):
	bods = cap.bodies
	ker,mun = bods[0],bods[1]
	
	x = [ker.R, 0., 0.]
	v = [0., 0., 175.]
	rock = bs.Rocket(x,v)
	
	if eles is None:
		eles = [[85.,60.],[60., 25],[25., 0.]]
	
	cnt1 = bs.Control( rock, ker, eles[0] )
	cnt2 = bs.Control( rock, ker, eles[1] )
	cnt3 = bs.Control( rock, ker, eles[2] )
	
	stage1 = bs.Stage( 16456. , 6150., 1193.16, 250563.22, cnt1 )
	stage2 = bs.Stage( 8646. , 4000., 663.15, 205574.89, cnt2 )
	stage3 = bs.Stage( 2856. , 1000., 43.84, 15125.53, cnt3 )
	rock.add_stage(stage3)
	rock.add_stage(stage2)
	rock.add_stage(stage1)
	
	cap.add_body(rock)
	
	return ker, mun, rock
