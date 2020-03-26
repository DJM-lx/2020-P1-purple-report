from joy.plans import Plan

class Move( Plan ):

    def __init__( self , app , simIX ):

        Plan.__init__( self, app )
        self.simIX = simIX
        self.REAL = app.REAL

        self.localNS = True
        self.angle = -8000
        self.speed = 0
        self.dist = 0
        self.dur = 2
        self.N = 20
        self.partP = 0

    def behavior( self ):

        if self.REAL == True:

            yield self.simIX.move( self.localNS , self.angle , self.speed , self.dist , self.partP )

        else:

            step = self.dist / float(self.N)
            dt = self.dur / float(self.N)

            for k in range(self.N):

                self.simIX.move(self.localNS , self.angle , self.speed , step , self.partP )
                yield self.forDuration(dt)

class Autonomous( Plan ):

    def __init__( self , app , simIX ,  sensor , moveP,firstWaypoint):

        Plan.__init__( self, app )
        self.moveP = moveP
	self.moveP.real = 1
	self.moveP.imag = 1
	self.a = 1
	self.moveP.dist = 10.0
	self.moveP.dur = 1
	self.threshold = 5
	self.prevsensor = sensor
	self.sensor = sensor
    self.coord=firstWaypoint;
    self.missedSensorCount=0
    self.missedSensorThreshold=4



    def behavior( self, sensor):

        self.movesim('x')
        self.nextmove('x')

     def movesim(self,direction):
	#check if a works as complex
	if direction ==('x' or 'X'):
		self.moveP.speed =  (a * self.moveP.dist * real(self.heading()))/self.moveP.dur
        #we need to do the local NS thing here.  How does that work?
        self.coord[0] += (a * self.moveP.dist * real(self.heading()))
	elif direction == ('y' or 'Y'):
    	self.moveP.speed =  (a * self.moveP.dist * imag(self.heading()))/self.moveP.dur
        #we need to do the local NS thing here.  How does that work?
        self.coord[1] += (a * self.moveP.dist * imag(self.heading()))
	self.lastsensor = sensor
	self.moveP.start()
    yield self.moveP.forDuration(self.moveP.dur)

	self.way = sensor.lastWaypoints[-1]
	self.x ,self.y = self.way[0]
	self.nextx , self.nexty = self.way[1]
	self.ts, self.f , self.b = sensor.lastSensor

     def heading(self):
	self.prevsensor_x = sensor
	self.way = sensor.lastWaypoints[-1]
	self.x ,self.y = self.way[0]
	self.nextx , self.nexty = self.way[1]
	self.ts, self.f_temp , self.b_temp = self.sensor.lastSensor
	if self.f_temp and self.b_temp:
        self.f=self.f_temp
        self.b=self.b_temp
		self.sense = (self.f + self.b) / 2.0
        self.missedSensorCount=0

	elif self.f_temp:
        self.f=self.f_temp
		self.sense = self.f

	elif self.b_temp:
        self.b=self.b_temp
		self.sense = self.b
    else:
        self.missedSensorCount += 1
    if self.missedSensorCount > self.missedSensorThreshold:
        return 1

	head = (self.nextx - self.x) + ((self.nexty - self.y)j)
	head_n= head / abs(head)
	if (self.sense < threshold): #move parallel to the line

		return head_n
	else: #move perpendicular to the line
		return imag(head_n)+real(head_n)j


      def checkDist2Line(self):
	self.ts, self.lastf, self.lastb = self.prevsensor.lastSensor
		if self.lastf and self.f:
			self.boolf = self.lastf < self.f
		elif self.lastb and self.b:
			self.boolb = self.lastb < self.b

    def nextMove(self,lastMove)

        checkDist2Line
        self.a=1
        if self.missedSensorCount < self.missedSensorThreshold:
            self.scan() #add scan as a function

        elif self.boolb and self.boolf:
            if lastMove == 'x':
                self.movesim('y')
                self.nextMove('y')
            else:
                self.movesim('x')
                self.nextMove('x')
        elif self.boolb or self.boolf:
            if lastMove == 'x':
                self.movesim('y')
                self.movesim('x')
                self.nextMove('x')
            else:
                    self.movesim('x')
                    self.movesim('y')
                    self.nextMove('y')
        else: # we are moving away from the line.  Backtrack.
            self.a=-2
            if lastMove=='x':
                self.movesim('y')
                self.nextMove('y')
            else:
                self.movesim('x')
                self.movesim('x')


    def scan(self):
       self.a = -1
       if lastMove == 'x'
            if(((self.coord[0] + (self.a * self.moveP.dist * real(self.heading())) < self.lowerx) || (self.coord[0] + (self.a * self.moveP.dist * real(self.heading())) > self.upperx) )
                self.a = 1
            self.movesim('x')
        else
            if(((self.coord[1] + (self.a * self.moveP.dist * imag(self.heading())) < self.lowery) || (self.coord[1] + (self.a * self.moveP.dist * imag(self.heading())) > self.uppery) )
                self.a = 1
            self.movesim('y')
        checkDist2Line
        if self.sensor <= 0
            self.scan()
