from joy.plans import Plan

class Move( Plan ):

    def __init__( self , app , simIX ):

        Plan.__init__( self, app )
        self.simIX = simIX

        self.localNS = True
        self.angle = -9000
        self.speed = 0
        self.dist = 0
        self.dur = 1

    def behavior( self ):

        yield self.simIX.move( self.localNS , self.angle , self.speed , self.dist )