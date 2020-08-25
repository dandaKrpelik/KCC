
class Engine:
	## assume singe propellant
	def __init__(self, part):
		self.engine = part
		self.propellant_name = part.engine.propellant_names[0]
		self.propellant = part.engine.propellants[0]
		
		self.initial_propellant_amount = self.propellant.current_amount
		
	@property
	def has_fuel(self):
		return self.engine.engine.has_fuel
	@property
	def fuel_ratio(self):
		return self.propellant.current_amount / self.initial_propellant_amount

class Vessel:
	stage= None
	
	def __init__(self, vessel):
		self.vessel = vessel
		parts = vessel.parts
		self.parts = parts

		self.mass = 0
		
		self.engines = {}
		for x in parts.all:
			self.mass += x.mass
			if not x.engine is None:
				engine = Engine(x)
				decouple_stage = x.decouple_stage
				
				if not decouple_stage in self.engines:
					self.engines.update({decouple_stage : []})
				
				self.engines[decouple_stage] += [engine]


		self.renew_stage()
		print('start stage ' + str(self.stage))

	def next_stage(self):
		self.vessel.control.activate_next_stage()
		if self.stage in self.engines:
			self.engines.pop(self.stage)
		self.renew_stage()
		print('start stage ' + str(self.stage))




	def renew_stage(self):
		self.stage = self.vessel.control.current_stage				
				
	def engines_have_fuel(self, next_stage):
		out = False
		for e in self.engines:
			if e == next_stage:
				for engine in self.engines[e]:
					out = out or engine.has_fuel
		return out

	def stage_fuel_level(self):
		pass
