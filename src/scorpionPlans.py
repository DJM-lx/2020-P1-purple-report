from joy.plans import Plan
from joy.decl import progress
from pdb import set_trace

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
        self.way=[]
        self.f=0
        self.lastf=0
        self.b=0
        self.lastb=0
        self.missedSensorCount=0
        self.sense=0
        self.lastSense=0



    def behavior( self ):

        if self.REAL == True:

            yield self.simIX.move( self.localNS , self.angle , self.speed , self.dist , self.partP )

        else:
            if self.noWay and self.sensor.lastWaypoints != (0,[]):
                self.firstWay=self.sensor.lastWaypoints[-1][0]
            step = self.dist / float(self.N)
            dt = self.dur / float(self.N)

            for k in range(self.N):

                self.simIX.move(self.localNS , self.angle , self.speed,self.dist)
                yield self.forDuration(dt)

    def getfirstWay( self ):
        if self.firstWay == 0:
            print('first waypoint not discovered')
            return 0
        else:
            return self.firstWay


    def updateSensors(self):
        # self.way = self.sensor.lastWaypoints[-1]
        # progress('lastWaypoints = '+ str(self.sensor.lastWaypoints)+'self.way = ' +str(self.way))
        # self.x ,self.y = self.way[0]
        # self.nextx , self.nexty = self.way[1]
        self.ts, self.f_temp , self.b_temp = self.sensor.lastSensor
        if self.f_temp and self.b_temp:
            self.lastf=self.f
            self.lastb=self.b
            self.f=self.f_temp
            self.b=self.b_temp
            self.lastSense=self.sense
            self.sense = (self.f + self.b) / 2.0
            self.missedSensorCount=0
        elif self.f_temp:
            self.lastf=self.f
            self.f=self.f_temp
            self.lastSense=self.sense
            self.sense = self.f
        elif self.b_temp:
            self.lastb=self.b
            self.b=self.b_temp
            self.lastSense=self.sense
            self.sense = self.b
        else:
            self.missedSensorCount += 1


class Autonomous( Plan ):

    def __init__( self , app , simIX ,  sensor , moveP):

        Plan.__init__( self, app )
        self.moveP = moveP
        self.ax= 1
        self.ay=-1
        self.normDist = 10.0
        self.moveP.dist=10
        self.moveP.speed = 1
        self.threshold = 5
        self.prevsensor = sensor
        self.sensor = sensor

        self.coord=[0,0]
        self.missedSensorCount=self.moveP.missedSensorCount
        self.missedSensorThreshold=50
        self.f=self.moveP.missedSensorCount
        self.lastf=self.moveP.lastf
        self.b=self.moveP.b
        self.lastb=self.moveP.lastb
        self.bIsCloser=False
        self.fIsCloser=False
        self.sense=self.moveP.sense
        self.lastSense=self.moveP.lastSense
        self.way=[]
        self.moveAlong=True
        self.i=0


    def behavior( self):


        progress('hi again')
        #set_trace()
        yield self.moveSim('x')
        yield self.nextMove('x')

    def moveSim(self,direction):
        '''
        Moves the robot in direction: (x or y) and a distance proportional to
        that direction's component in heading()
        '''
        progress('start moveSim')
        #direction=direction.upcase()
        if direction =='x':

            self.moveP.dist =  self.ax * self.normDist * self.heading().real
            self.moveP.localNS = False
            progress('auto move x'+str(self.moveP.dist))
            # self.coord[0] += self.ax * self.moveP.dist * self.heading().real
        elif direction == 'y':
            self.moveP.dist =  self.ay * self.normDist * self.heading().imag
            self.moveP.localNS = True
            progress('auto move y'+str(self.moveP.dist))
            # self.coord[1] += self.ay * self.moveP.dist * self.heading().imag
        self.moveP.dur=abs(self.moveP.dist)/self.moveP.speed
        progress('dur= '+ str(self.moveP.dur))
        self.moveP.start()
        yield self.moveP.forDuration(self.moveP.dur)

        self.way = self.sensor.lastWaypoints[-1]
        self.x ,self.y = self.way[0]
        self.nextx , self.nexty = self.way[1]
        self.ts, self.f , self.b = self.sensor.lastSensor

    def heading(self):
        '''
        Returns a unit vector in the complex plane that points :
            towards the next waypoint if it is within a threshold distance from
            the line

            perpendicular to the line if outside of the threshold

        if sensor data appears to be unreliable, it defaults to pointing in the
        real direction
        '''
        # self.prevsense = self.sense
        self.way = self.sensor.lastWaypoints[-1]
        progress('lastWaypoints = '+ str(self.sensor.lastWaypoints)+'self.way = ' +str(self.way))
        self.x ,self.y = self.way[0]
        self.nextx , self.nexty = self.way[1]
        # self.ts, self.f_temp , self.b_temp = self.sensor.lastSensor
        # if self.f_temp and self.b_temp:
        #     self.f=self.f_temp
        #     self.b=self.b_temp
        #     self.sense = (self.f + self.b) / 2.0
        #     self.missedSensorCount=0
        # elif self.f_temp:
        #     self.f=self.f_temp
        #     self.sense = self.f
        # elif self.b_temp:
        #     self.b=self.b_temp
        #     self.sense = self.b
        self.moveP.updateSensors()



        if self.missedSensorCount > self.missedSensorThreshold:
            return 1
        head = (self.nextx - self.x) + (self.nexty - self.y)*1j
        head_n= head / abs(head)
        if (self.sense < self.threshold): #move parallel to the line
        # if self.moveAlong==True:
            print('heading || = ' + str(head_n))
            return head_n
        else: #move perpendicular to the line
            print('heading |_ = ' + str(head_n.imag+head_n.real*1j))
            return head_n.imag+head_n.real*1j


    def checkDist2Line(self):
        '''
        update self.bIsCloser and self.fIsCloser.  They will remain unchanged if
         there is not sensor data
        '''
        if self.lastf and self.f:
            self.fIsCloser = self.lastf < self.f
        elif self.lastb and self.b:
            self.bIsCloser = self.lastb < self.b

    def nextMove(self,lastMove):
        '''
        chooses between moving the robot in +x,-x,+y,or-y.  Decision is made
        based on whether the robot moved closer or farther from the line in the
        last step.  self.a should be 1 to move in the direction of heading() or
        -2 to backtrack
        '''

        while True:
            progress('starting nextMove('+str(self.i)+'). Last Move : '+ lastMove)
            self.checkDist2Line()
            self.i+=1
            if self.missedSensorCount > self.missedSensorThreshold:
                yield self.scan(lastMove)

            if self.bIsCloser and self.fIsCloser:
                progress('closer,closer')
                if lastMove == 'x':
                    yield self.moveSim('y')
                    lastMove='y'
                else:
                    yield self.moveSim('x')
                    lastMove='x'
            elif self.bIsCloser or self.fIsCloser:
                progress('one sensor is closer')
                if lastMove == 'x':
                    yield self.moveSim('y')
                    yield self.moveSim('x')
                    lastMove='x'
                else:
                        yield self.moveSim('x')
                        yield self.moveSim('y')
                        lastMove='y'
            else: # we are moving away from the line.  Backtrack.
                progress('farther')
                if lastMove=='x':
                    self.ax=-self.ax
                    yield self.moveSim('y')
                    lastMove='y'
                else:
                    self.ay=-self.ay
                    yield self.moveSim('x')
                    lastMove='x'



    def scan(self,lastMove):
        progress('Entering scan..')
        absDist=0
        self.moveP.localNS = False

        while True:
            absDist += 1
            self.moveP.dist=absDist
            self.moveP.dur=absDist/self.moveP.speed
            self.moveP.start()
            yield self.moveP.forDuration(self.moveP.dur)
            if self.missedSensorCount < self.missedSensorThreshold:
                return
            self.moveP.localNS = True
            absDist += 1
            self.moveP.dist=absDist
            self.moveP.dur=absDist/self.moveP.speed
            self.moveP.start()
            yield self.moveP.forDuration(self.moveP.dur)
            if self.missedSensorCount < self.missedSensorThreshold:
                return
            self.moveP.localNS=False
            absDist += 1
            self.moveP.dist=-absDist
            self.moveP.dur=absDist/self.moveP.speed
            self.moveP.start()
            yield self.moveP.forDuration(self.moveP.dur)
            if self.missedSensorCount < self.missedSensorThreshold:
                return
            self.moveP.localNS=True
            absDist += 1
            self.moveP.dist=-absDist
            self.moveP.dur=absDist/self.moveP.speed
            self.moveP.start()
            yield self.moveP.forDuration(self.moveP.dur)
            if self.missedSensorCount < self.missedSensorThreshold:
                return

        # self.a = -1
        # if lastMove == 'x':
        #     if(((self.coord[0] + (self.a * self.moveP.dist * real(self.heading())) < self.lowerx) or (self.coord[0] + (self.a * self.moveP.dist * real(self.heading())) > self.upperx))):
        #         self.a = 1
        #     yield self.moveSim('x')
        # else:
        #     if(((self.coord[1] + (self.a * self.moveP.dist * imag(self.heading())) < self.lowery) or (self.coord[1] + (self.a * self.moveP.dist * imag(self.heading())) > self.uppery))):
        #         self.a = 1
        #     yield self.moveSim('y')
        # self.checkDist2Line()
        # if self.sensor <= 0:
        #     self.scan()
