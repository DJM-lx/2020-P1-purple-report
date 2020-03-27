# file robotSimulator.py simulates a robot in an arena

import sys
import os

if 'pyckbot/hrb/' not in sys.path:
  sys.path.append(os.path.expanduser('~/pyckbot/hrb/'))

from sensorPlanTCP import SensorPlanTCP
from robotSimIX import SimpleRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT, ref 
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis

if '-r' in sys.argv:
  print('INITIALIZING PHYSICAL ROBOT')
  REAL = True
  ARG = 2
  from scorpionReIX import ScorpionRobotSim, RobotSimInterface

else:
  print('INITIALIZING SIMULATED ROBOT')
  REAL = False
  ARG = 1
  from scorpionSimIX import ScorpionRobotSim, RobotSimInterface

from scorpionPlans import Move, Autonomous
from particleFilter import ParticleFilter
# IMPORT MORE PLANS WHEN MORE ARE WRITTEN

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
    self.REAL = REAL

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
    self.robSim = ScorpionRobotSim(fn=None, app=self)
    self.moveP = Move(self,self.robSim)
    self.autoP = Autonomous(self, self.robSim, self.sensor, self.moveP) # NEED TO FILL WITH REQUIRED PARAMETERS AND CHANGE IN PLANS SCRIPT
    self.partP = ParticleFilter(self, self.sensor, self.moveP)
    self.autoOn = False
    self.nowUpdate = False
    self.good = False
    if ref[0,1] < ref[2,1]: #26 is the bottom left tag
        self.lowerx = ref[0,1]
        self.upperx = ref[2,1]
    else:
        self.upperx = ref[0,1]
        self.lowerx = ref[2,1]
    self.lowery = ref[0,0]
    self.uppery = ref[5,0]
        

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
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
  
  def fakeauto(self):
    pass

  def calcdist( self ):
    self.way = self.sensor.lastWaypoints[-1]
    if not self.way:
      self.way = self.sensor.lastWaypoints[-1]
      self.nowUpdate = True
      self.dist = 0
      self.localNS = True
      return
    elif not self.good and self.way:
      self.lastway = self.way[0]
      self.nextway = self.way[1]
      print (str(self.way))
      print (str(self.lastway))
      print(str(self.nextway))
      self.x1, self.y1 = self.lastway
      self.x2, self.y2 = self.nextway
      self.good = True
    if self.nowUpdate and self.way:
      self.lastway = self.way[0]
      self.nextway = self.way[1]
      print (str(self.way))
      print (str(self.lastway))
      print(str(self.nextway))
      self.x1, self.y1 = self.lastway
      self.x2, self.y2 = self.nextway
    

    stepval = 1
    x1 = self.x1 
    x2 = self.x2
    y1 = self.y1
    y2 = self.y2
    self.dist = 0
    if x2 - x1 > 0:
      temp = (x2 - x1) % 50
      self.x1 += temp
      self.dist = -1 * temp 
      self.localNS = False
      print("XXXXXXXXXXXXX")
      print("x1 " ,self.x1 ," y2 ", self.x2)
      return
    elif x2 - x1 < -0:
      temp = (x2 - x1) % 50
      self.x1 -= temp
      self.dist = temp
      self.localNS = False
      print("-----------------XXXXXXXXXXXXX")
      return
    if y2 - y1 > 0:
      temp = (y2 - y1) % 50
      self.y1 += temp
      self.dist =  (temp) 
      self.localNS = True
      print("YYYYYYYYYYYYYYYYYY")
      print("y1 " ,self.y1 ," y2 ", self.y2)
      print("dist" , self.dist)
    elif y2 - y1 < 0:
      temp = (y2 - y1) % 50
      self.y1 -= temp
      self.dist = -1 * (temp ) 
      self.localNS = True
      print("---------YYYYYYYYYYYY")
    else:
      self.dist = 0
      self.localNS = True
      print ("MOVING >>>>>><<<<<<<<<< X1 <<<<<<<<>>>>>>>> Y1 <<<<<<>>>>>>>")

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
    if evt.type == KEYDOWN:
      if evt.key == K_a and not self.autoP.isRunning():
        self.autoP.start()
        # self.calcdist()
        # self.moveP.dist = self.dist * 5
        # self.moveP.localNS = self.localNS
        # if self.dist != 0:
        #   self.moveP.speed = self.moveP.dist/ 5
        #   self.moveP.start()
        return progress("(say) Autonomous")
      elif evt.key == K_UP and not self.moveP.isRunning():
        self.moveP.localNS = True
        self.moveP.dist = -50.0
        self.moveP.partP = self.partP

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Move forward")
      elif evt.key == K_DOWN and not self.moveP.isRunning():
        self.moveP.localNS = True
        self.moveP.dist = 50.0
        self.moveP.partP = self.partP

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Move back")
      elif evt.key == K_LEFT and not self.moveP.isRunning():
        self.moveP.localNS = False
        self.moveP.dist = -50.0
        self.moveP.partP = self.partP

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Turn left")
      elif evt.key == K_RIGHT and not self.moveP.isRunning():
        self.moveP.localNS = False
        self.moveP.dist = 50.0
        self.moveP.partP = self.partP

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Turn right")
    # # elif self.autoOn is True and not self.autoP.isRunning():
    # #     exit()
    # #     self.autoP.start()
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

  if '-r' in sys.argv:
      sys.argv.remove('-r')
      modules = {'count':4, 'names':{0x04:'CCWS', 0x0C: 'CCWM', 0x10: 'CWS', 0xD0: 'CWM'}, 'fillMissing':True,'required':[0x04,0x0C,0x10,0xD0]}
  else:
      modules = None
      

  if len(sys.argv)>ARG:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, robot=modules, cfg={'windowSize' : [160,120]})
  app.run()