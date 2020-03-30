from joy.plans import Plan
from joy.decl import progress

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

        self.sensor = sensor
        self.reading = False
        self.first_waypoint = None
        self.previous_waypoint = None
        self.waypoint_size = 0

    def behavior( self ):

        if self.REAL == True:

            yield self.simIX.move( self.localNS , self.angle , self.speed , self.dist )

        else:

            step = self.dist / float( self.N )
            dt = self.dur / float( self.N )

            for k in range( self.N ):

                self.simIX.move( self.localNS , self.angle , self.speed , self.dist )
                yield self.forDuration( dt )

                ts , w = self.sensor.lastWaypoints

                if ts:

                    if not self.reading:

                        self.first_waypoint = w[ 0 ]

                        self.reading = True

                        self.waypoint_size = len( w )
                    
                    if len( w ) < self.waypoint_size:

                        self.previous_waypoint = self.first_waypoint
                        #yield progress( str( self.previous_waypoint ) )
                        self.reading = False
                        break




class Autonomous( Plan ):

    def __init__( self , app , simIX ,  sensor , moveP ):

        Plan.__init__( self, app )

        self.sensor = sensor
        self.moveP = moveP

        self.epsilonNS = 24
        self.epsilonWE = 5

        self.odd = True

    def behavior( self ):

        ts , w = self.sensor.lastWaypoints

        if ts:
            
            yield self.search(  )

            while( True ):

                if not self.moveP.reading:
                    
                    yield self.goToWaypoint(  )

                    self.odd = not self.odd
                
                else:

                    yield self.search( )

        else:

            yield progress( 'no valid reading' )
    
    def search( self ):

        alpha_distance = 0.0

        while True:

            self.moveP.localNS = False

            self.moveP.dist = -1.0 - alpha_distance
            self.moveP.dur = abs( self.moveP.dist ) / 2.0 
            self.moveP.N = int( self.moveP.dur * 10 )

            yield self.moveP.start(  )
            yield self.forDuration( 2 * self.moveP.dur )

            if not self.moveP.reading:
                break

            self.moveP.localNS = True

            self.moveP.dist = 1.0 + alpha_distance
            self.moveP.dur = abs( self.moveP.dist ) / 2.0 
            self.moveP.N = int( self.moveP.dur * 10 )

            yield self.moveP.start(  )
            yield self.forDuration( 2 * self.moveP.dur )

            if not self.moveP.reading:
                break

            alpha_distance += 1.0

            self.moveP.localNS = False

            self.moveP.dist = 1.0 + alpha_distance
            self.moveP.dur = abs( self.moveP.dist ) / 2.0 
            self.moveP.N = int( self.moveP.dur * 10 )

            yield self.moveP.start(  )
            yield self.forDuration( 2 * self.moveP.dur )

            if not self.moveP.reading:
                break

            self.moveP.localNS = True

            self.moveP.dist = -1.0 - alpha_distance
            self.moveP.dur = abs( self.moveP.dist ) / 2.0 
            self.moveP.N = int( self.moveP.dur * 10 )

            yield self.moveP.start(  )
            yield self.forDuration( 2 * self.moveP.dur )

            if not self.moveP.reading:
                break

            alpha_distance += 1.0

    def goToWaypoint( self ):

        ts , w = self.sensor.lastWaypoints
        
        #yield progress( str( w ) )

        if self.odd:

            y = self.moveP.first_waypoint[1] - w[0][1]
            x = self.moveP.first_waypoint[0] - w[0][0]
        
        else:

            y = w[0][1] - w[1][1]
            x = w[0][0] - w[1][0]

        yield progress( str( y ) )
        yield progress( str( x ) )

        self.moveP.localNS = False

        self.moveP.dist = y #/ 6.0
        self.moveP.dur = abs( self.moveP.dist ) / 2.0 
        self.moveP.N = int( self.moveP.dur * 10 )

        yield self.moveP.start(  )
        yield self.forDuration( 2 * self.moveP.dur )

        if self.moveP.reading:
            
            self.moveP.localNS = True

            self.moveP.dist = -1 * ( x )#/ 16.0 )
            self.moveP.dur = abs( self.moveP.dist ) / 2.0 
            self.moveP.N = int( self.moveP.dur * 10 )

            yield self.moveP.start(  )
            yield self.forDuration( 2 * self.moveP.dur )