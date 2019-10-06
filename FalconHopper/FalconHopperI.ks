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
local HTp is 0.09. // Hover Throttle P
local HTi is 0.001. // Hover Throttle I
local HTd is 0.25. // Hover Throttle D
local HTPID is PIDLOOP(HTp,HTi,HTd,-.01,.01).
local Tmin is 0.1. // minimum throttle setting
local AUTOPILOT is true. // program will run until this is switched off
local LAUNCH_AMSL is ROUND(ship:ALTITUDE,3). // AMSL of the control module
local LAUNCH_AGL is ROUND(MAX(0.001,(ALTITUDE-GEOPOSITION:TERRAINHEIGHT)),3). // AGL of the control module
local HAGL is 250. // TARGET HOVER ALT METERS AGL

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
    print "AGL:" + centreMassRadalt(LAUNCH_AGL) at (TERMINAL:WIDTH - 14,TERMINAL:HEIGHT - 3).
    print "AMSL:" + ROUND(ship:ALTITUDE - LAUNCH_AGL,3) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 2).
  }

}

// this is the main sequence of the flight plan
function doFlightTriggers {

  when true THEN {
    // Tower Phase
    print "# TOWER PHASE #".
    lock throttle to towerThrottle().
    lock steering to up.
    when centreMassRadalt(LAUNCH_AGL) > LAUNCH_AGL THEN {
      // cleared tower
      print "# HOVER PHASE #".
      lock steering to up. // hoverSteering().
      set throttle to Tmin.
      until not AUTOPILOT {
        set throttle to hoverThrottle(). // TODO lock?
      }
    }
  }

}

function hoverThrottle {
  local thrott is throttle + HTPID:update(time:seconds, centreMassRadalt(LAUNCH_AGL)).
  if (thrott < Tmin) {
    set thrott to Tmin.
  } else if (thrott > 1) {
    set thrott to 1.
  }
  return thrott.
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
