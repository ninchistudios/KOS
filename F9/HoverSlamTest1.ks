// LANDING ZONES
// KSC LZ1: Lat -000.032492 Long -074.663904 0 AGL 67m AMSL
// KSC LZ2: Lat -000.032570 Long -074.642213 0 AGL 67m AMSL
// KSC FIWLT: Lat -000.149711 Long -074.013484 AGL 741.7 AMSL 0

// F9 AGs:
// AG1: Engine mode (9/3/1)
// AG7: Grid Fins

// launch
// ascend to 250m
// hover for 30 sec
// descend
// land conservatively at same spot

@LAZYGLOBAL OFF.

// include utilities once and only once
copypath("0:/common/Utils", "").
runoncepath("Utils").

print " ".
print "#####################################".
print "# FALCON 9 HOVERSLAM TEST PROGRAM 1 #".
print "#####################################".
print " ".

// CONFIGURE FLIGHT
local TCOUNT is 3. // T-Minus countdown
local TIGNITE is 1. // Ignite at T-Minus...
local HTp is 0.05. // Hover Throttle P
local HTi is 0.1. // Hover Throttle I
local HTd is 0.15. // Hover Throttle D
local HTPID is PIDLOOP(HTp,HTi,HTd,-.1,.05). // adjust last two values for throttle speed
local Tmin is 0.1. // minimum throttle setting
local AUTOPILOT is true. // program will run until this is switched off
local HOVER_MODE is false. // flag for the main loop
local SLAM_MODE is false. // time to do a hoverslam
local LAUNCH_AMSL is ROUND(ship:ALTITUDE,3). // AMSL of the control module
local LAUNCH_AGL is ROUND(MAX(0.001,(ALTITUDE-GEOPOSITION:TERRAINHEIGHT)),3). // AGL of the control module
local HAGL is 250. // TARGET HOVER ALT METERS AGL
local PREDICTED is 0. // predicted radalt with current accel + vel
local GAGL is 500. // engage gear below on descent
local vAngle is 0. // angle from ship up to surface up
local Fg is 0. // force of gravity on the ship
local AGL is 0. // current AGL of the nozzles
local tweak is -120. // hoverslam AGL tweak
neutraliseRoll().
set kuniverse:TimeWarp:MODE to "PHYSICS".

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
  // print "dV check -100:" + desiredVv(-100).
  // print "dV check -50:" + desiredVv(-50).
  // print "dV check 0:" + desiredVv(0).
  // print "dV check 50:" + desiredVv(50).
  // print "dV check 100:" + desiredVv(100).
  // print "TWR check 0:" + desiredTWR(10,10).
  lock vAngle to VANG(ship:facing:forevector, ship:up:forevector).
  lock Fg to (body:mu / body:radius^2) * mass.
  lock AGL to baseRadalt(LAUNCH_AGL).
  if ADDONS:TR:AVAILABLE {
      PRINT "Trajectories available.".
  } else {
      PRINT "ERROR: Trajectories is not available.".
  }
  doCountdown(TCOUNT, TIGNITE, Tmin).
}

// loops while inflight
function doMain {

  doPreservedTriggers().
  doFlightTriggers().

  until not AUTOPILOT {
    doDebug().
    if (HOVER_MODE) {
      // maintains throttle locked to hover
    }
    if (SLAM_MODE) {
      doHoverslam().
    }
  }

}

// run last
function doFinalise {
  lock THROTTLE to 0.
  // TODO clear flightplan
  // set ship:control:neutralize to true.
  // print "### PROGRAM COMPLETE ###".
  until false {
    wait 1.
  }
}

// the main sequence of the flight plan
function doFlightTriggers {

  when true THEN {
    // Tower Phase
    print "# TOWER PHASE #".
    lock throttle to towerThrottle().
    lock steering to up. // up

    doSafeStage().
    wait 1.
    rcs off.
    // when AGL > LAUNCH_AGL THEN {
    when true THEN {
      set warp to 3.
      // cleared tower
      print "# HOVER PHASE #".
      lock steering to up. // hoverSteering().
      lock throttle to hoverThrottle().
      set HOVER_MODE to true.
      when (ship:mass < (2 * ship:drymass)) then {
        set warp to 0.
        wait 2.
        // hoverslam
        set SLAM_MODE to true.
        set HOVER_MODE to false.
        when ship:verticalSpeed < 0 then {
          lock steering to srfRetrograde.
          rcs on.
          AG7 on.
          when distanceToGround() < GAGL then {
             gear on.
          }
        }

      }
    }
  }
}

function doPreservedTriggers {

}

function hoverThrottle {
  // thrott = TWRd x Fg / AVAILABLETHRUST
  // TWRd = (0.00012 * dV^3) + (0.000514286 * dV^2 + (0.003 * dV) +0.998286)
  // dV = (0.0732601 * dH^3) - (17.326 * dH)
  local dH is AGL - HAGL. // delta between desired height and actual height
  print "dH:" + ROUND(dH,3) at (TERMINAL:WIDTH - 13,TERMINAL:HEIGHT - 13).
  local dV is desiredVv(dH). // desired surface velocity based on dH
  print "dV:" + ROUND(dV,3) at (TERMINAL:WIDTH - 13,TERMINAL:HEIGHT - 14).
  local TWRd is desiredTWR(dV, ship:verticalspeed). // desired TWR
  print "TWRd:" + ROUND(TWRd,3) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 15).
  // local thrott is Fg * Tf / COS(vAngle) / AVAILABLETHRUST.
  local vthrott is TWRd * Fg / max(1,AVAILABLETHRUST). // throttle assuming vertical
  print "vthrott:" + ROUND(vthrott,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 16).
  local thrott is vthrott / COS(vAngle).
  print "thrott:" + ROUND(thrott,3) at (TERMINAL:WIDTH - 17,TERMINAL:HEIGHT - 17).
  if (thrott < Tmin) {
    set thrott to Tmin.
    // TODO if we're still accelerating up we need to shutdown some engines
  } else if (thrott > 1) {
    set thrott to 1.
  }
  return thrott.
}

// basic function to calculate the desired vertical speed based on the distance from the target height
function desiredVv {
  parameter dH1.
  return max(-20,min(10,(-0.00000533333 * dH1^3) + (0.000000000000000000243714 * dH1^2) - (.0466667 * dH1))).
}

// basic function to calculate the desired TWR based on the desired vertical speed and current vertical speed
function desiredTWR {
  parameter dv0,v0.
  local dv1 is dv0 - v0.
  // return max(0.8,min(1.3,(0.00012 * dV1^3) + (0.000514286 * dV1^2 + (0.003 * dV1) +0.998286))).
  return max(0,min(1.3,(0.038118 + (0.961882 * (constant:e ^ (0.0770805 * dv1)))))).
}

function doHoverslam {
  // stopping distance formula
  // sd = v^2 / 2a
  // lock steering to heading(180, 88).
  toggle AG1. // 9>3 engines
  // lock throttle to 1.
  // wait 8.
  lock throttle to Tmin.
  lock pct to stoppingDistance() / distanceToGround().
  wait until pct > 1.
  lock throttle to max(pct,Tmin).
  // stage.
  // AG8 on. // single engine
  // neutraliseRoll().
  // local throt is Tmin.
  // lock throttle to throt.
  //wait until ship:verticalSpeed > 0.
  //lock throttle to 0.
  until ship:status = "LANDED" {
    print "DTG:" + ROUND(distanceToGround, 1) at (TERMINAL:WIDTH - 14,TERMINAL:HEIGHT - 3).
    //set throt to min(1,max(0,(((0.635/(1+constant:e^(5-1.5*(ALTITUDE-GEOPOSITION:TERRAINHEIGHT))))+((ALTITUDE-GEOPOSITION:TERRAINHEIGHT)/min(-1,(verticalspeed))))+(abs(verticalspeed)/(availablethrust/mass))))).
    wait 0.
  }

  print "SLAM DONE".
  lock throttle to 0.
  AG7 off. // gridfins
  set SLAM_MODE to false.
  lock steering to up.
}

function distanceToGround {
  return altitude - body:geopositionOf(ship:position):terrainHeight - LAUNCH_AGL.
}

function stoppingDistance {
  local grav is constant():g * (body:mass / body:radius^2).
  local maxDeceleration is (ship:availableThrust / ship:mass) - grav.
  return ship:verticalSpeed^2 / (2 * maxDeceleration).
}

function predictedRadalt {
  local s0 is AGL. // AGL of the nozzles
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

function neutraliseRoll {
  SET STEERINGMANAGER:ROLLPID:KP TO 0. // set roll rate to 0.
  SET STEERINGMANAGER:ROLLPID:KI TO 0. // set roll rate to 0.
}

function doDebug {
  // print "X Accel:" + ROUND(ship:sensors:acc:x,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 6).
  // print "Y Accel:" + ROUND(ship:sensors:acc:y,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 5).
  // print "Z Accel:" + ROUND(ship:sensors:acc:z,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 4).
  print "Vv:" + ROUND(ship:verticalspeed,3) at (TERMINAL:WIDTH - 13,TERMINAL:HEIGHT - 6).
  print "vAngle:" + ROUND(vAngle,3) at (TERMINAL:WIDTH - 17,TERMINAL:HEIGHT - 5).
  print "Fg:" + ROUND(Fg,3) at (TERMINAL:WIDTH - 13,TERMINAL:HEIGHT - 4).
  print "AGL:" + ROUND(AGL, 1) at (TERMINAL:WIDTH - 14,TERMINAL:HEIGHT - 3).
  print "AMSL:" + ROUND(ship:ALTITUDE - LAUNCH_AGL,3) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 2).
  print "Mass:" + ROUND(ship:mass,1) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 7).
  print "DryMass:" + ROUND(ship:drymass,1) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 8).
  print "WetMass:" + ROUND(ship:wetmass,1) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 9).
}
