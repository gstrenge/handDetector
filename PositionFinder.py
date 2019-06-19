class Position:

	def __init__(self, _x, _y):
		self.x = _x
		self.y = _y
		
		self.oldx = _x
		self.oldy = _y
		
		self.vx = 0
		self.vy = 0
		
		self.oldvx = 0
		self.oldvy = 0
		
		self.ax = 0
		self.ay = 0
		
		
	def update(self,newx, newy, dt):
		self.oldx = self.x
		self.oldy = self.y
		
		self.x = newx
		self.y = newy
		
		self.oldvx = self.vx
		self.oldxy = self.vy
		
		self.vx = (self.x-self.oldx)/float(dt)
		self.vy = (self.y-self.oldy)/float(dt)
		
		#self.ax = (self.vx-self.oldvx)/float(dt)
		#self.ay = (self.vy-self.oldvy)/float(dt)
		
	def futurePosition(self,time):
		future_x = self.x + (self.vx)*(time) #+ (.5)*(self.ax)*(time**2)
		future_y = self.y + (self.vy)*(time) #+ (.5)*(self.ay)*(time**2)
		return int(future_x),int(future_y)#Position(future_x, future_y)
		
	def velocityMag(self):
		return ((self.vx * self.vx) + (self.vy * self.vy))**.5
		
	def velocitySqrMag(self):
		return ((self.vx * self.vx) + (self.vy * self.vy))
		
	
	
		
