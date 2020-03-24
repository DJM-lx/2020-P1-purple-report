from joy.plans import Plan

class ParticleFilter( Plan ):

    def __init__( self , app , sensor , moveP ):
        self.sensor = sensor

    def behavior( self ):
        self.sensor.lastWaypoints[-1]

    def update( self , dist ):
        pass

    def getWaypoints( self ):
        pass

    def getRobotCoords( self ):
        pass

    def useScan( self ):
        pass