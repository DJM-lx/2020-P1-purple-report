from joy.plans import Plan

class Move( Plan ):

    def __init__( self , app , simIX ,sensor):

        Plan.__init__( self, app )
        self.simIX = simIX
        self.REAL = app.REAL

        self.localNS = True
        self.angle = -8000
        self.speed = 0
        self.dist = 0
        self.dur = 2
        self.N = 20
        self.noWay=True
        self.firstWay=0
        self.sensor=sensor

    def behavior( self ):

        if self.REAL == True:

            yield self.simIX.move( self.localNS , self.angle , self.speed , self.dist , self.partP )

        else:
            if self.noWay and self.sensor.lastWaypoints != (0,[]):
                self.firstWay=self.sensor.lastWaypoints[-1][0]
            step = self.dist / float(self.N)
            dt = self.dur / float(self.N)

            for k in range(self.N):

                self.simIX.move(self.localNS , self.angle , self.speed)
                yield self.forDuration(dt)

    def getfirstWay( self ):
        if self.firstWay == 0:
            print('first waypoint not discovered')
            return None
        else:
            return self.firstWay



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
        self.coord=0
        self.missedSensorCount=0
        self.missedSensorThreshold=4



    def behavior( self):

        self.movesim('x')
        self.nextMove('x')

    def movesim(self,direction):
        if direction ==('x' or 'X'):
            self.moveP.speed =  (self.a * self.moveP.dist * real(self.heading()))/self.moveP.dur
            self.moveP.localNS = False
            self.coord[0] += (self.a * self.moveP.dist * real(self.heading()))
        elif direction == ('y' or 'Y'):
            self.moveP.speed =  (self.a * self.moveP.dist * imag(self.heading()))/self.moveP.dur
            self.moveP.localNS = True
            self.coord[1] += (self.a * self.moveP.dist * imag(self.heading()))
        self.lastsensor = self.sensor
        self.moveP.start()
        yield self.moveP.forDuration(self.moveP.dur)

        self.way = self.sensor.lastWaypoints[-1]
        self.x ,self.y = self.way[0]
        self.nextx , self.nexty = self.way[1]
        self.ts, self.f , self.b = self.sensor.lastSensor

    def heading(self):
        self.prevsensor_x = self.sensor
        self.way = self.sensor.lastWaypoints[-1]
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
        head = (self.nextx - self.x) + (self.nexty - self.y)*1j
        head_n= head / abs(head)
        if (self.sense < self.threshold): #move parallel to the line
            return head_n
        else: #move perpendicular to the line
            return imag(head_n)+real(head_n)*1j


    def checkDist2Line(self):
        self.ts, self.lastf, self.lastb = self.prevsensor.lastSensor
        if self.lastf and self.f:
            self.boolf = self.lastf < self.f
        elif self.lastb and self.b:
            self.boolb = self.lastb < self.b

    def nextMove(self,lastMove):

        self.checkDist2Line()
        self.a=1
        if self.missedSensorCount < self.missedSensorThreshold:
            self.scan(lastMove) 

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


    def scan(self,lastMove):
        self.a = -1
        if lastMove == 'x':
            if(((self.coord[0] + (self.a * self.moveP.dist * real(self.heading())) < self.lowerx) or (self.coord[0] + (self.a * self.moveP.dist * real(self.heading())) > self.upperx))):
                self.a = 1
            self.movesim('x')
        else:
            if(((self.coord[1] + (self.a * self.moveP.dist * imag(self.heading())) < self.lowery) or (self.coord[1] + (self.a * self.moveP.dist * imag(self.heading())) > self.uppery))):
                self.a = 1
            self.movesim('y')
        self.checkDist2Line()
        if self.sensor <= 0:
            self.scan()
