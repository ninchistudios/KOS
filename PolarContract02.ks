@LAZYGLOBAL OFF.

print " ".
print "#############################################".
print "# POLAR CONTRACT 02 (PROBO) ORBITAL PROGRAM #".
print "#############################################".
print " ".

// CONFIGURE LAUNCH
local TGT_APO is 90000. // target orbit to be circularised
local TGT_INCL is 0. // target orbital inclination
local TCOUNT is 3. // T-Minus countdown
local Kp is 0.35. //0.01
local Ki is 0.15. // 0.006
local Kd is 0.15. // 0.006
local TGT_Q is 99. // ideal max Q, 26=375dV

// RUN
doSetup().
doMain().
doFinalise().

// runs once
function doSetup {
  global MY_VESSEL is SHIP. // safes against vehicle switch
  global LAST_T is 0. // last available thrust
  global AUTOPILOT is true. // program will run until this is switched off
  global LAUNCH_ALT is ROUND(MY_VESSEL:ALTITUDE,1). // alt of the control module
  global PID_THROTT is 0. // the PID controller's throttle setting
  lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa. // dynamic pressure in kPa
  global PID is PIDLOOP(Kp, Kp, Kd).
  set PID:SETPOINT TO TGT_Q.
  set PID:MAXOUTPUT to 1.
  set PID:MINOUTPUT to -1.
  // surface key flight data
  print "Target Orbit: " + TGT_APO + "m".
  print "Target Orbital Inclination: " + TGT_INCL + " degrees".
  print "Launch Altitude: " + LAUNCH_ALT + "m".
  doCountdown(TCOUNT).
}

// loops while program executing
function doMain {

  doPreservedTriggers().
  doAscentTriggers().

  until not AUTOPILOT {
    print "Q:" + ROUND(MY_Q,3) at (TERMINAL:WIDTH - 12,TERMINAL:HEIGHT - 2).
    // short delay after 0 thrust staging before throttle comes up (prevent boom)
    // maintain steering to achieve target orbital inclination
    // create maneuver to circularise, via genetic algo
    // calc burn time of maneuver
    // execute circ burn
  }

}

// set up ascent triggers
function doAscentTriggers {
  // go ballistic once apo is at target orbit
  when MY_VESSEL:APOAPSIS >= TGT_APO then {
    print "# BALLISTIC PHASE #".
    lock THROTTLE to 0.
    // wait until above atmo to create node
    when MY_VESSEL:ALTITUDE >= 70000 then {
      createCircNode().
    }
  }
  // blow accel-safe modules outside the atmo
  when MY_VESSEL:ALTITUDE > 60000 THEN {
    deployAccelSafe().
  }
  // steering profiles
  when MY_VESSEL:ALTITUDE < 2 * LAUNCH_ALT THEN {
    // Tower Phase
    print "# TOWER PHASE #".
    lock THROTTLE to ascentThrottle().
    lock steering to heading(TGT_INCL, 89.9).
    when MY_VESSEL:ALTITUDE > 2 * LAUNCH_ALT THEN {
      // cleared tower
      print "# ATMO PENETRATION PHASE #".
      lock steering to heading(ascentHeading(), ascentPitch()).
    }
  }
}

function createCircNode {
  local circ is list().
}

function calculateStartTime {
  parameter mnv.
  return time:seconds + mnv:eta -maneuverBurnTime(mnv) / 2.
}

function maneuverBurnTime {
  parameter mnv.
  local dV is mnv:deltaV:mag.
  local isp is 0.
  local myEngines is 0.
  list engines in myEngines.
  for en in myEngines {
    if en:ignition and not en:flameout {
      set isp to isp + (en:isp * (en:AVAILABLETHRUST / MY_VESSEL:AVAILABLETHRUST)).
    }
  }
  local mf is MY_VESSEL:mass / constant:e^(dV / (isp * constant:g0)).
  local fuelFlow is MY_VESSEL:AVAILABLETHRUST / (isp * constant:g0).
  local t is (MY_VESSEL:mass - mf) / fuelFlow.
  return t.
}

function doPreservedTriggers {
  // stage if needed throughout the program
  when stageNeeded() then {
    doSafeStage().
    preserve.
  }
  // debug - prints maneuver time on AG9 toggle
  when AG9 then {
    if hasnode {
      local n is nextnode.
      print "BurnTime:" + ROUND(maneuverBurnTime(n),1) at (TERMINAL:WIDTH - 19,TERMINAL:HEIGHT - 3).
      print "BurnStart:" + ROUND(calculateStartTime(n),1) at (TERMINAL:WIDTH - 20,TERMINAL:HEIGHT - 4).
    }
    AG9 off.
    preserve.
  }
}

function executeManeuver {
  parameter utime, radial, normal, prograde.
  local mnv is node(utime, radial, normal, prograde).
  // addManeuverToFlightPlan(mnv).
  // local startTime is calculateStartTime(mnv).
  // wait until time:seconds > startTime - 10.
  // lockSteeringAtManeuverTarget(mnv).
  // wait until time:seconds > startTime.
  // lock throttle to 1.
  // wait until isManeuverComplete(mnv).
  // lock throttle to 0.
  // removeManeuverFromFlightPlan(mnv).
}

function ascentThrottle {
  set PID_THROTT to PID_THROTT + PID:UPDATE(TIME:SECONDS, MY_Q).
  if (PID_THROTT < 0) {
    set PID_THROTT to 0.
  } else if (PID_THROTT > 1) {
    set PID_THROTT to 1.
  }
  WAIT 0.001.
  return PID_THROTT.
}

function ascentHeading {
  return TGT_INCL.
}

function ascentPitch {
  local tp is min(89.9,217.86 - 18.679 * ln(MY_VESSEL:ALTITUDE)).
  print "PITCH:" + ROUND(tp,3) at (TERMINAL:WIDTH - 16,TERMINAL:HEIGHT - 1).
  return tp.
}

// accel-safe deployments (e.g. fairings) should be set to AG1
function deployAccelSafe {
  AG1 ON.
  print "# ACCEL-SAFE MODULES DEPLOYED #".
}

// orbit-safe deployments (e.g. science) should be set to AG2
function deployOrbitSafe {
  AG2 ON.
  print "# ORBIT-SAFE MODULES DEPLOYED #".
}

// stage with a delay until ready
function doSafeStage {
  wait until STAGE:READY.
  print "# STAGING #".
  STAGE.
  set LAST_T to MY_VESSEL:AVAILABLETHRUST.
}

// have we experienced a drop in available thrust?
function stageNeeded {
  local t is MY_VESSEL:AVAILABLETHRUST.
  if (t = 0 or t < (LAST_T - 10)) {
    // uh oh, loss of available thrust
    set LAST_T to t.
    return true.
  } else {
    set LAST_T to t.
    return false.
  }
}

// T-minus countdown
function doCountdown {
  local parameter t.
  print "# T MINUS".
  from {local c is t.} until c = 0 step {set c to c - 1.} do {
    print "# ... " + c.
    WAIT 1.
  }
}

// run last
function doFinalise {
  lock THROTTLE to 0.
  deployOrbitSafe().
  // TODO clear flightplan
  set MY_VESSEL:control:neutralize to true.
  print "### PROGRAM COMPLETE ###".
}
