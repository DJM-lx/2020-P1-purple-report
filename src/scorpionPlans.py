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

    def behavior( self ):
        
        if self.REAL == True:

            yield self.simIX.move( self.localNS , self.angle , self.speed , self.dist )

        else:

            step = self.dist / float(self.N)
            dt = self.dur / float(self.N)

            for k in range(self.N):

                self.simIX.move(self.localNS , self.angle , self.speed , step)
                yield self.forDuration(dt)

class Autonomous( Plan ):

    def __init__( self , app , simIX ,  sensor , moveP):
        
        Plan.__init__( self, app )
        pass
            

    def behavior( self ):
        
        pass