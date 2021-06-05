from ece163.Constants.VehiclePhysicalConstants as VPC

class Target:
	def __init__(self, state, deliveryAirSpeed = 25.0, deliveryAltitude = 100.0):
		# MVP
		self.pn = state.pn
		self.pe = state.pe
		self.deliveryAirSpeed = deliveryAirSpeed
		self.deliveryAltitude = deliveryAltitude
		
		# post MVP
		'''
		self.pd = state.pd
		self.u = state.u
		self.v = state.v
		self.w = state.w
		'''
		return
		
		
	def reset(self):
		# MVP
		self.pn = state.pn
		self.pe = state.pe
		self.deliveryAirSpeed = VPC.InitialSpeed
		self.deliveryAltitude = VPC.InitialDownPosition
		
		'''
		# post MVP
		self.pd = state.pd
		self.u = state.u
		self.v = state.v
		self.w = state.w
		'''
		return
	
	def setDeliveryAirSpeed(self, airSpeed):
		self.deliveryAirSpeed = airSpeed
		return
		
	def setDeliveryAltitude(self, altitude):
		self.deliveryAltitude = altitude
		return
	
	def setPosition(self, Pn, Pe):
		self.pn = newPn
		self.pe = newPe
		#self.pd = newPd
		return
		
	def getPosition(self):
		return self.pe, self.pn
		
	def setVelocity(self, u, v):
		self.u = u
		self.v = v
		return
		
	def getVelocity(self):
		return self.u, self.v
	
	'''
	def Update(self, state):
		# take previous position and move it forawrd the distance it
		# it would move in one step
	'''