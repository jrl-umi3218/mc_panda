#!/usr/bin/python

import rtm
import socket
# import robot
import sys

# from rtm import *
from rtm import connectPorts
from rtm import narrow
from rtm import isJython

# import OpenHRP
from OpenHRP import *
if isJython():
    from OpenHRP.RobotHardwareServicePackage import *

rtm.nsport = 2809
rtm.nshost = "localhost"

use_udp = @USE_UDP@

try:
    import cnoid.Corba
    from cnoid.grxui import *
    orb = cnoid.Corba.getORB()
    rtm.orb = orb
    if rtm.nshost is None:
        rtm.nshost = socket.gethostname()
    nsloc = "corbaloc:iiop:%s:%d/NameService" % (rtm.nshost, rtm.nsport)
    print(nsloc)
    rtm.initCORBA()
    sys.argv = []
    inChoreonoid = True
except Exception:
    rtm.initCORBA()
    inChoreonoid = False

def connectComps():
    connectPorts(rh.port("q"), [sh.port("currentQIn")])
    # for the kinematics mode
    if kinematics_mode == 1:
        connectPorts(sh.port("qOut"), rh.port("qIn"))
        connectPorts(sh.port("baseTformOut"), rh.port("baseTformIn"))
    #
    if not rh.port("baseTform"):
        connectPorts(sh.port("qOut"), servo.port("angleRef"))
    else:
        connectPorts(sh.port("baseTformOut"), rh.port("baseTform"))


def createComps():
    global ms, rh, rh_svc, sh, sh_svc, tk_svc, st, kf, log, log_svc, servo
    global ep_svc, mc, mc_ctrl

    rh = rtm.findRTC("pandaController(Robot)0")
    servo = rtm.findRTC("PDcontroller0")

    ms = rtm.findRTCmanager()

    ms.load("KalmanFilter")
    kf = ms.create("KalmanFilter")

    ms.load("StateHolder")
    sh = ms.create("StateHolder")
    sh_svc = narrow(sh.service("service0"), "StateHolderService")
    tk_svc = narrow(sh.service("service1"), "TimeKeeperService")

    ms.load("DataLogger")
    log = ms.create("DataLogger")
    log_svc = narrow(log.service("service0"), "DataLoggerService")

    if use_udp:
      ms.load("MCUDPSensors")
      mc = ms.create("MCUDPSensors")

      ms.load("MCUDPControl")
      mc_ctrl = ms.create("MCUDPControl")
    else:
      ms.load("MCControl")
      mc = ms.create("MCControl")

def activateComps():
    if use_udp:
      rtm.serializeComponents([rh, kf, log, mc_ctrl, sh, mc])
    else:
      rtm.serializeComponents([rh, kf, log, mc, sh])
    rh.start()
    kf.start()
    sh.start()
    log.start()
    mc.setProperty("robot", "@ROBOT_NAME@")
    mc.start()
    if use_udp:
      mc_ctrl.setProperty("robot", "@ROBOT_NAME@")
      mc_ctrl.start()

def init(hostname=socket.gethostname()):
    global ms, simulation_mode, kinematics_mode

    ms = rtm.findRTCmanager()

    rh = rtm.findRTC("pandaController(Robot)0")
    servo = rtm.findRTC("PDcontroller0")
    simulation_mode = 1
    if rh.port("baseTformIn"):
        kinematics_mode = 1
    else:
        kinematics_mode = 0

    print("creating components")
    createComps()

    print("connecting components")
    connectComps()

    print("activating components")
    activateComps()

    print("initialized successfully")


def startMCControl():
    global mc, mc_ctrl
    mc.setProperty("is_enabled", "1")
    if use_udp:
      mc_ctrl.setProperty("is_enabled", "1")

def connectMCControl():
    connectPorts(rh.port("q"), mc.port("qIn"))
    connectPorts(rh.port("wristsensorA"), mc.port("@SENSOR_PORT_A@"))
    if use_udp:
      connectPorts(mc_ctrl.port("qOut"), sh.port("qIn"))
    else:
      connectPorts(mc.port("qOut"), sh.port("qIn"))


init()
connectMCControl()
startMCControl()
