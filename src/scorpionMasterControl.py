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
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis

if '-r' in sys.argv:
  print('INITIALIZING PHYSICAL ROBOT')
  from scorpionReIX import ScorpionRobotSim, RobotSimInterface

else:
  print('INITIALIZING SIMULATED ROBOT')
  # FILL IN THE REST FOR SIMULATED ROBOT

from scorpionPlans import Move
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
      if evt.key == K_UP and not self.moveP.isRunning():
        self.moveP.localNS = True
        self.moveP.dist = -50.0

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Move forward")
      elif evt.key == K_DOWN and not self.moveP.isRunning():
        self.moveP.localNS = True
        self.moveP.dist = 50.0

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Move back")
      if evt.key == K_LEFT and not self.moveP.isRunning():
        self.moveP.localNS = False
        self.moveP.dist = -50.0

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Turn left")
      if evt.key == K_RIGHT and not self.moveP.isRunning():
        self.moveP.localNS = False
        self.moveP.dist = 50.0

        self.moveP.speed = self.moveP.dist/self.moveP.dur
        self.moveP.start()
        return progress("(say) Turn right")
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
      modules = {'count':4, 'names':{0x0C:'CCWS', 0x4D: 'CCWM', 0x08: 'CWS', 0x14: 'CWM'}, 'fillMissing':True,'required':[0x0c,0x4d,0x08,0x14]}
  else:
      modules = None

  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
  app.run()