@LAZYGLOBAL OFF.

// include utilities once and only once
runoncepath("Utils").

print " ".
print "##################################".
print "# FALCON HOPPER I FLIGHT PROGRAM #".
print "##################################".
print " ".

// CONFIGURE FLIGHT
local TCOUNT is 3. // T-Minus countdown
local HTp is 0.05. // Hover Throttle P
local HTi is 0.1. // Hover Throttle I
local HTd is 0.2. // Hover Throttle D
local HTPID is PIDLOOP(HTp,HTi,HTd,-.1,.05). // adjust last two values for throttle speed
local Tmin is 0.1. // minimum throttle setting
local AUTOPILOT is true. // program will run until this is switched off
local HOVER_MODE is false. // flag for the main loop
local LAUNCH_AMSL is ROUND(ship:ALTITUDE,3). // AMSL of the control module
local LAUNCH_AGL is ROUND(MAX(0.001,(ALTITUDE-GEOPOSITION:TERRAINHEIGHT)),3). // AGL of the control module
local HAGL is 250. // TARGET HOVER ALT METERS AGL
local PREDICTED is 0. // predicted radalt with current accel + vel

// RUN
doSetup().
doMain().
doFinalise().

// runs once
function doSetup {
  set HTPID:SETPOINT TO HAGL.
  // surface key flight data
  print "Target Hover: " + HAGL + "m AGL".
  print "Launch AMSL: " + LAUNCH_AMSL + "m".
  print "Launch AGL: " + LAUNCH_AGL + "m".
  doCountdown(TCOUNT).
}

function doMain {

  doPreservedTriggers().
  doFlightTriggers().

  until not AUTOPILOT {
    print "X Accel:" + ROUND(ship:sensors:acc:x,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 6).
    print "Y Accel:" + ROUND(ship:sensors:acc:y,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 5).
    print "Z Accel:" + ROUND(ship:sensors:acc:z,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 4).
    print "AGL:" + baseRadalt(LAUNCH_AGL) at (TERMINAL:WIDTH - 14,TERMINAL:HEIGHT - 3).
    print "AMSL:" + ROUND(ship:ALTITUDE - LAUNCH_AGL,3) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 2).
    if (HOVER_MODE) {
        set throttle to hoverThrottle(). // TODO lock?
    }
  }

}

function realG {
  parameter sensorG.
  return sensorG + body:mu / (altitude + body:radius)^2.
}

// this is the main sequence of the flight plan
function doFlightTriggers {

  when true THEN {
    // Tower Phase
    print "# TOWER PHASE #".
    lock throttle to towerThrottle().
    lock steering to up.
    rcs on.
    stage.
    wait 3.
    stage.
    when baseRadalt(LAUNCH_AGL) > LAUNCH_AGL THEN {
      // cleared tower
      print "# HOVER PHASE #".
      lock steering to up. // hoverSteering().
      set HOVER_MODE to true.
    }
  }

}

function hoverThrottle {
  local thrott is throttle + HTPID:update(time:seconds, predictedRadalt()).
  if (thrott < Tmin or PREDICTED > (1.2 * HAGL)) {
    set thrott to Tmin.
  } else if (thrott > 1) {
    set thrott to 1.
  }
  return thrott.
}

function predictedRadalt {
  local s0 is baseRadalt(LAUNCH_AGL). // AGL of the nozzles
  print "s0:" + ROUND(s0,3) at (TERMINAL:WIDTH - 13,TERMINAL:HEIGHT - 11).
  local u is ship:verticalspeed. // the current velocity, -ve up
  print "u:" + ROUND(u,3) at (TERMINAL:WIDTH - 12,TERMINAL:HEIGHT - 10).
  local a is ship:sensors:acc:y. // this appears to be very unreliable
  print "a:" + ROUND(a,3) at (TERMINAL:WIDTH - 12,TERMINAL:HEIGHT - 9).
  // t is the time to v = 0 under the current a: v=u+at
  local t is (0 - u) / a.
  print "t:" + ROUND(t,3) at (TERMINAL:WIDTH - 12,TERMINAL:HEIGHT - 8).
  // distance to peak: s=ut+1/2at^2
  local s is (u * t) + (0.5 * a * (t^2)).
  print "s:" + ROUND(t,3) at (TERMINAL:WIDTH - 12,TERMINAL:HEIGHT - 7).
  set PREDICTED to s + s0. // TODO is this correct?
  print "predicted:" + ROUND(PREDICTED,1) at (TERMINAL:WIDTH - 20,TERMINAL:HEIGHT - 12).
  return PREDICTED.
}

function doPreservedTriggers {

}

// run last
function doFinalise {
  lock THROTTLE to 0.
  // TODO clear flightplan
  set ship:control:neutralize to true.
  print "### PROGRAM COMPLETE ###".
}
