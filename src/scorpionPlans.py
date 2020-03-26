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

    def __init__( self , app , simIX ,  sensor , moveP):

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
    self.coord=



    def behavior( self, sensor):
		#check diff in old and new sensor values





		#end moves robot
		#keep track of distance/sensor values
        self.movesim('x')
        self.nextmove('x')

     def movesim(self,direction):
	#check if a works as complex
	if(direction='x')
		self.moveP.speed =  (a * self.moveP.dist * real(self.heading()))/self.moveP.dur

	else
	self.moveP.speed =  (a * self.moveP.dist * imag(self.heading()))/self.moveP.dur
	self.lastsensor = sensor
	self.moveP.start()
        yield self.moveP.forDuration(self.moveP.dur)

	self.way = sensor.lastWaypoints[-1]
	self.x ,self.y = self.way[0]
	self.nextx , self.nexty = self.way[1]
	self.ts, self.f , self.b = sensor.lastSensor

     def heading(self):
	self.prevsensor = sensor
	self.way = sensor.lastWaypoints[-1]
	self.x ,self.y = self.way[0]
	self.nextx , self.nexty = self.way[1]
	self.ts, self.f , self.b = sensor.lastSensor
	if self.f and self.b:
		self.sense = (self.f + self.b) / 2.0
	elif self.f:
		self.sense = self.f
		self.doscan = True
	elif self.b:
		self.doscan = True
		self.sense = self.b
	else:
		self.sense = None
		self.doscan = True
		#Do Something Else/ add
	head = (self.nextx - self.x) + ((self.nexty - self.y) * j)
	head_n= head / abs(head)
	if (self.sense < threshold):

		return head_n
	else:
		return imag(head_n)+real(head_n)j


      def checkDist2Line(self):
	self.ts, self.lastf, self.lastb = self.prevsensor.lastSensor
		if self.lastf and self.f:
			self.boolf = self.lastf < self.f
		if self.lastb and self.b:
			self.boolb = self.lastb < self.b

    def nextMove(self,lastMove)
        checkDist2Line
        self.a=1
        if self.sensor <=0     #change implementation of sensor to make this work:
            self.scan() #add scan as a function

        elif self.boolb and self.boolf
            if lastMove == 'x'
                self.movesim('y')
                self.nextMove('y')
            else
                self.movesim('x')
                self.nextMove('x')
        elif self.boolb or self.boolf
            if lastMove == 'x'
                self.movesim('y')
                self.movesim('x')
                self.nextMove('x')
            else
                    self.movesim('x')
                    self.movesim('y')
                    self.nextMove('y')
        else # we are moving away from the line.  Backtrack.
            self.a=-2
            if lastMove == 'x'
                self.movesim()
