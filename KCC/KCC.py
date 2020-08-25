from CommandCentre import *
from Missions import *

# initialize the CC thread
CC = CommandCentre()
CC.start()

def stopCC(save = False):
	if save:
		CC.sc.save(CC.save_game_name)
	CC.stop()
	CC.join()
	
	
def start_test_mission():
	vessel = CC.sc.active_vessel
	#mission = test_mission(vessel)
	mission = m_orbit1(vessel)
	CC.register_mission(mission)
	mission.start()
	return mission


