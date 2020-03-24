import sys
import os

if 'pyckbot/hrb/' not in sys.path:
  sys.path.append(os.path.expanduser('~/pyckbot/hrb/'))

from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, newaxis, mean, exp, sum, sqrt, any, isnan
from numpy.linalg import svd
from numpy.random import randn
from waypointShared import *

from joy.plans import Plan

from pdb import set_trace as DEBUG

def tags2list( dic ):
    """
    Convert a dictionary of tags into part of a list for JSON serialization

    INPUT:
      dic -- dictionary mapping tag id to 4x2 corner location array

    OUTPUT:
      list to be concatenated into JSON message
    """
    return [
        {
          'i' : k,
          'p': [ list(row) for row in v ]
        }
        for k,v in dic.items()
    ]

def findXing(a,b):
  """
  Find the crossing point of two lines, represented each by a pair of
  points on the line

  INPUT:
    a -- 2x2 -- two points, in rows
    b -- 2x2 -- two points, in rows

  OUTPUT: c -- 2 -- a point, as an array
  """
  a = asfarray(a)
  b = asfarray(b)
  # The nullspace of this matrix is the projective representation
  # of the intersection of the lines. Each column's nullspace is
  # one of the lines
  X = c_[a[1]-a[0],b[0]-b[1],a[0]-b[0]].T
  if X.ndim is not 2:
    DEBUG()
  Q = svd(X)[0]
  # Last singular vector is basis for nullspace; convert back from
  # projective to Cartesian representation
  q = Q[:2,2]/Q[2,2]
  c = q[0]*(a[1]-a[0])+a[0]
  return c

class RobotSimInterface( object ):
  """
  Abstract superclass RobotSimInterface defines the output-facing interface
  of a robot simulation.

  Subclasses of this class must implement all of the methods
  """
  def __init__(self, fn=None):
    """
    INPUT:
      fn -- filename / None -- laser log name to use for logging simulated
          laser data. None logged if name is None

    ATTRIBUTES:
      tagPos -- 4x2 float array -- corners of robot tag
      laserAxis -- 2x2 float array -- two points along axis of laser
      waypoints -- dict -- maps waypoint tag numbers to 4x2 float
          arrays of the tag corners
    """
    pass

  def refreshState( self ):
    """<<pure>> refresh the value of self.tagPos and self.laserAxis"""
    print("<<< MUST IMPLEMENT THIS METHOD >>>")

  def getTagMsg( self ):
    """
    Using the current state, generate a TagStreamer message simulating
    the robot state
    """
    return ""

  def logLaserValue( self, now ):
    """
    Using the current state, generate a fictitious laser pointer reading
    INPUT:
      now -- float -- timestamp to include in the message

    OUTPUT: string of human readable message (not what is in log)
    """
    return ""

class ScorpionRobotSim( RobotSimInterface ):
    def __init__(self, app=None, *args, **kw):
        RobotSimInterface.__init__(self, *args, **kw)

        self.initPos = 0
        self.initSpeed = 0

        self.app = app
        self.modules = [app.robot.at.CCWS, app.robot.at.CCWM, app.robot.at.CWS, app.robot.at.CWM]

        self.idxCCWS = 0
        self.idxCCWM = 1
        self.idxCWS = 2
        self.idxCWM = 3

        #self.modules[self.idxCCWS].set_mode('servo')
        self.modules[self.idxCCWS].set_pos(self.initPos)
        self.modules[self.idxCCWM].set_mode('motor')
        self.modules[self.idxCCWM].set_speed(self.initSpeed)

        #self.modules[self.idxCWS].set_mode('servo')
        self.modules[self.idxCWS].set_pos(self.initPos)
        self.modules[self.idxCWM].set_mode('motor')
        self.modules[self.idxCWM].set_speed(self.initSpeed)

    def move(self, localNS, angle, speed, dist , partP):
      if localNS == True:
        self.modules[self.idxCWS].set_pos(-angle)
        self.modules[self.idxCWM].set_speed(speed)

        yield self.app.moveP.forDuration(dist/speed)

        self.modules[self.idxCWS].set_pos(self.initPos)
        self.modules[self.idxCWM].set_speed(self.initSpeed)

      else:
        self.modules[self.idxCCWS].set_pos(angle)
        self.modules[self.idxCCWM].set_speed(speed)

        yield self.app.moveP.forDuration(dist/speed)

        self.modules[self.idxCCWS].set_pos(self.initPos)
        self.modules[self.idxCCWM].set_speed(self.initSpeed)

    def refreshState(self):
        pass