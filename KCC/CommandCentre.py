import krpc
import numpy as np
import matplotlib.pyplot as plt
import time
import threading

## main control class - runs in background, stores all data, is controlled from python command line
class CommandCentre(threading.Thread):
	KSPconnection = None	## connection to the KSP
	client = None
	_alive = True		## for stopping the thread
	
	save_game_name = None
	
	active_missions = []	##keep track of active missions
	active_vessels = []	## keep track of vessels on active missions
	
	def __init__(self, save_game = 'KCC_save', conn_name = 'KCC'):
		threading.Thread.__init__(self)			## CC is a background thread to allow python command line control
		
		print('initializing KCC server')
		self.save_game_name = save_game
		
		self.KSPconnection = krpc.connect(name=conn_name)		## init KPS connection with conn_name tag
		self.krpc = self.KSPconnection.krpc ## active game - holds information about current screen
		self.sc = self.KSPconnection.space_center ## space center
		
	def run(self):
		while self._alive:
			#print('centre active at ' + str(time.ctime(time.time())))
			if self.active_vessels:
				print('KCC '+str(self.active_vessels))
			time.sleep(1)

	def stop(self):
		self._alive = False



	def register_mission(self, mission):
		self.active_missions += [ mission ]
