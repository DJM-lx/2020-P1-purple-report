
from sensorPlanTCP import SensorPlanTCP
from robotSimIX import SimpleRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis

autonomous = False

class MoveForward(Plan):
  """
  Plan simulates robot moving forward or back over a period of time.
  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.move(step)
      yield self.forDuration(dt)

class Turn(Plan):
  """
  Plan simulates robot turning over a period of time.
  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Angle to turn [rad]
    self.ang = .1
    # Duration of travel [sec]
    self.dur = 3.0
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    # Compute rotation step
    dt = self.dur / float(self.N)
    step = self.ang / float(self.N)
    for k in range(self.N):
      s.turn(step)
      yield self.forDuration(dt)


class Auto(Plan):
  """
  Plan simulates robot turning over a period of time.
  (MODIFY THIS FOR YOUR ROBOT)
  """
  def __init__(self,app,simIX, sensor):
    Plan.__init__(self,app)
    self.simIX = simIX
    self.sensor = sensor
    # Angle to turn [rad]
    self.ang = 0
    # Duration of travel [sec]
    self.dur = 1
    # Number of intermediate steps
    self.N = 3

  def behavior(self,val):
    s = self.simIX
    # Compute rotation step
    dt = self.dur / float(self.N)
    step = self.ang / float(self.N)
    for k in range(self.N):
      s.turn(step)
      yield self.forDuration(dt)

class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.
     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """
  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    """
    Initialize the simulator
    """
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      )
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)
    # ADD pre-startup initialization here, if you need it
    autonomous = False

  def onStart( self ):
    """
    Sets up the JoyApp and configures the simulation
    """
    ### DO NOT MODIFY ------------------------------------------
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
    self.sensor.start()
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now
    ### MODIFY FROM HERE ------------------------------------------
    self.robSim = SimpleRobotSim(fn=None)
    self.moveP = MoveForward(self,self.robSim)
    self.turnP = Turn(self,self.robSim)
    self.way = self.sensor.lastWaypoints[-1]
    progress("Way %s" % str(self.sensor.lastWaypoints))
    #self.nextway = self.way[] 
    self.firstbool = 0
    self.nextway = []

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
      progress( "Sensor: %4d %4d" % (ts,self.T0)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
      self.waydist = ts
      self.way = w
      print (str(self.way))
      #initializing == GOOD
      if self.firstbool == 1:
	if not self.sensors.lastWaypoints:
		self.firstbool = 2
		self.way = self.sensors.lastWaypoints[-1]
		print (str(self.way))
		self.lastway = self.way[0]
		self.nextway = self.way[1]
		print (str(self.way))
		print (str(self.lastway))
		print(str(self.nextway))
      if self.firstbool == 2:
	if not self.nextway:
		#for updateing waypoints === GOOD
	      	if self.nextway != self.way[1]:
			print ("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
			self.lastway = self.way[0]
			self.nextway = self.way[1]
			print (str(self.lastway))
			print(str(self.nextway))
			self.x1, self.y1 = self.lastway 
			self.x2, self.y2 = self.nextway
			self.update= 1

	

    else:
      progress( "Waypoints: << no reading >>" )

  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    #### DO NOT MODIFY --- it WILL break the simulator
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg.encode("ascii"), self.srvAddr)

  def calcdist( self ):
	#initializing == GOOD
	if not self.nextway or not self.lastway:
		self.lastway = self.way[0]
		self.nextway = self.way[1]
		print (str(self.way))
		print (str(self.lastway))
		print(str(self.nextway))
		self.x1, self.y1 = self.lastway
		self.x2, self.y2 = self.nextway
		self.update = 1
	x1 = self.x1 
	x2 = self.x2
	y1 = self.y1
	y2 = self.y2
	self.update = 0
	stepval = 1
	
	#BADDD -> VALUES SET TO MOVE != DIST ACTUALLY MOVED === REWORK
	if x2 - x1 > 0:
		temp = (x2 - x1) 
		self.xf = temp * 1.0j
		print("XXXXXXXXXXXXX")
	elif x2 - x1 < -0:
		temp = (x2 - x1) 
		self.xf = (temp * 1.0j)
		print("-----------------XXXXXXXXXXXXX")
	if y2 - y1 > 0:
		temp = (y2 - y1) 
		self.yf = -1 * (temp) 
		print("YYYYYYYYYYYYYYYYYY")
	elif y2 - y1 < -0:
		temp = (y2 - y1) 
		self.yf = 1 * (temp ) 
		print("---------YYYYYYYYYYYY")
	else:
		self.yf = 0
		self.xf = 0
	self.x1 = x1
	self.y1 = y1
	print ("MOVING >>>>>><<<<<<<<<< X1 <<<<<<<<>>>>>>>> Y1 <<<<<<>>>>>>>")
	print( str(self.xf) + "__" + str(self.yf))


  def onEvent( self, evt ):
    #### DO NOT MODIFY --------------------------------------------
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus():
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame():
      self.emitTagMessage()
    #### MODIFY FROM HERE ON ----------------------------------------
    #initialize == Good
    if self.firstbool == 1:
	if not self.way:
		self.firstbool = 2
		self.way = self.sensors.lastWaypoints[-1]
		self.lastway = self.way[0]
		self.nextway = self.way[1]
		self.x1, self.y1 = self.lastway 
		self.x2, self.y2 = self.nextway
		print (str(self.way))
		print (str(self.lastway))
		print(str(self.nextway))
    if evt.type == KEYDOWN:
        self.showSensors()
        if evt.key == K_a and not self.moveP.isRunning():
            #HMMMMMMMMM
            self.calcdist()
	    self.moveP.dist = self.xf * 5
            self.moveP.start()
	    self.moveP.dist = self.xf * 5
            self.moveP.start()
            return progress("(say) Automatic mode activating")
        if evt.key == K_UP and not self.moveP.isRunning():
            self.moveP.dist = 50.0
            self.moveP.start()
	    if self.firstbool == 0:
            	self.firstbool = 2
            return progress("(say) Translate forward")
        elif evt.key == K_DOWN and not self.moveP.isRunning():
            self.moveP.dist = -50.0
            self.moveP.start()
	    print(str(self.disttest))
	    if self.firstbool == 0:
            	self.firstbool = 2
            return progress("(say) Translate back")
        if evt.key == K_LEFT and not self.turnP.isRunning():
            self.moveP.dist = 50.0j
            self.moveP.start()
	    if self.firstbool == 0:
            	self.firstbool = 2
            return progress("(say) Translate left")
        if evt.key == K_RIGHT and not self.turnP.isRunning():
            self.moveP.dist = -50.0j
            self.moveP.start()
	    if self.firstbool == 0:
            	self.firstbool = 2
            return progress("(say) Translate right")
	if evt.key == K_y and not self.turnP.isRunning():
            self.moveP.dist = 0
	    self.ang = .1
            self.turnP.start()
	    if self.firstbool == 0:
            	self.firstbool = 2
            return progress("(say) Translate right")
	if evt.key == K_t and not self.turnP.isRunning():
            self.turnP.dist = 0
	    self.turnP = -.1
            self.turnP.start()
	    if self.firstbool == 0:
            	self.firstbool = 2
            return progress("(say) Translate right")
    ### DO NOT MODIFY -----------------------------------------------
    else:# Use superclass to show any other events
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  print("""
  Running the robot simulator
  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer.
  """)
  import sys
  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
  app.run()
