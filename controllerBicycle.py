#!/usr/bin/python
import time
import serial
import numpy as np
from math import *

class Simulation:
  def __init__(self):
    self.db = False
    self.strData = ""
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #Control output
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    self.Mz = 0
    self.Dc = 0
    self.dc = 0
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #Model parameters
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    self.Lf = 1.17
    self.Lr = 1.43
    self.Cf = 1.81
    self.Cr = 1.68
    self.Bf = 7.2
    self.Br = 11.0
    self.Dr = 8394.0
    self.Df = 8854.0
    self.m = 1480.0
    self.Jz = 2300.0
    self.mu = 0.9
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #Control gain
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    self.M1 = 10.0
    self.M2 = 3.5
    self.M3 = 1.0
    self.M4 = 1.0
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #Variables for integration
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    self.v1Ant = 0.0
    self.v2Ant = 0.0
    self.tm = 0.001
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    #Serial port configuration
    #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    self.port = serial.Serial(
      '/dev/ttyS0',
      baudrate = 115200,
      parity = serial.PARITY_NONE,
      stopbits = serial.STOPBITS_ONE,
      bytesize = serial.EIGHTBITS,
      timeout = 1
    )

  def startSimulation(self, _cmd):
    while(True):
      _strCmd = self.port.readline()
      if(self.db == True):
        print _strCmd
      if(_cmd == _strCmd):
        print "Ready"
        break

  def sendOutSystem(self):
    self.port.write('%4.20g,%4.20g\n'%(self.Mz, self.dc))
    if (self.db == True):
      print 'Send:{!r},{!r}'.format(self.Mz, self.dc)

  def control(self, _vx, _vy, _wz, _vyRef, _wzRef, _dd):
    alphar = -(_vy - self.Lr * _wz)/_vx
    alphaf0 = _dd - (_vy + self.Lf * _wz)/_vx

    phir = sin(self.Cr * atan(self.Br * alphar))
    phif0 = sin(self.Cf * atan(self.Bf * alphaf0))

    B = np.matrix([[self.mu*self.Df/self.m, 0], [self.mu*self.Df*self.Lf/self.Jz, 1/self.Jz]])

    eVy = _vy - _vyRef
    eWz = _wz - _wzRef

    dv1 = -self.M3 * np.sign(eVy)
    dv2 = -self.M4 * np.sign(eWz)
    v1Act = self.v1Ant + self.tm*dv1
    v2Act = self.v2Ant + self.tm*dv2

    C = np.matrix([[-self.M1*sqrt(abs(eVy))*np.sign(eVy)+v1Act], [-self.M2*sqrt(abs(eWz))*np.sign(eWz)+v2Act]])
    self.v1Ant = v1Act
    self.v2Ant = v2Act

    #[-M1 * sqrt(abs(eVy)) * sign(eVy) + v(1); -M2*sqrt(abs(eWz))*sign(eWz) + v(2)];
    invB = np.linalg.inv(B)
    u = invB*C
    #Matrix.Multiply((float*)B,(float*)C,N,N,N,(float*)u);
    self.Mz = u[1];
    Dc = u[0];
      
    if(Dc > (1-phif0)):
      Dc = (1-phif0)

    if(Dc < (-1 - phif0)):
      Dc = -1 - phif0
      #Mz1=-(Df*mu*phif0*Lf-Dr*mu*phir*Lr)-mu*Df*Lf*Dcreal+Jz*dxref(2);

    phif = Dc + phif0;
    self.dc = -_dd + (_vy + self.Lf * _wz) / _vx + (1/self.Bf)*tan((1/self.Cf)*asin((phif)));

  def extractVar(self, _str):
    if((_str.startswith('|') == True) and (_str.endswith('\n') == True) and (_str != '|start\n')):
      _startStr = _str.find('|')
      _endStr = _str.find(',')
      _endStrData = _str.find('\n')
      vxStr = _str[_startStr + 1: _endStr]

      _startStr = _endStr + 1
      _d1 = _str[_startStr: _endStrData]
      _endStr = _d1.find(',')
      vyStr = _d1[0: _endStr]

      _startStr = _endStr + 1
      _d2 = _d1[_startStr: _endStrData]
      _endStr = _d2.find(',')
      wzStr = _d2[0: _endStr]

      _startStr = _endStr + 1
      _d3 = _d2[_startStr: _endStrData]
      _endStr = _d3.find(',')
      vyRefStr = _d3[0: _endStr]

      _startStr = _endStr + 1
      _d4 = _d3[_startStr: _endStrData]
      _endStr = _d4.find(',')
      wzRefStr = _d4[0: _endStr]

      _startStr = _endStr + 1
      ddStr = _d4[_startStr: _endStrData]

      vx = float(vxStr)
      vy = float(vyStr)
      wz = float(wzStr)
      vyref = float(vyRefStr)
      wzref = float(wzRefStr)
      dd = float(ddStr)

      self.control(vx, vy, wz, vyref, wzref, dd);
      if(self.db == True):
        print '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}'.format(vxStr,
          vx, vyStr, vy, wzStr, wz, vyRefStr, vyref, wzRefStr, wzref, ddStr,
          dd)

control = Simulation()
control.startSimulation('|start\n')

while (True):
  control.strData = control.port.readline()
  if(control.strData == ""):
    print "Finished"
    break
  control.extractVar(control.strData)
  control.sendOutSystem()