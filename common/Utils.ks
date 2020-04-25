@LAZYGLOBAL OFF.

local LAST_T is 0.
local TICK is 0.

function towerThrottle {
  wait 0.001.
  return 1.
}

function hoverSteering {
  return up.
}

// note this changes with fuel burn, so... TODO need to figure this out.
function baseRadalt {
  parameter launchRadAlt.
  // wait 0.001.
  return MAX(0.00001,((ALTITUDE-GEOPOSITION:TERRAINHEIGHT)-launchRadAlt)).
}

function doCircularization {
  parameter vessel, est_dv.
  local circ is list(time:seconds + eta:APOAPSIS, 0, 0, est_dv).
  until false {
    local oldScore is score(circ).
    set circ to improve(circ).
    if oldScore <= score(circ) {
      break.
    }
  }
  executeManeuver(circ, vessel).
}

function score {
  parameter data.
  local mnv is node(data[0], data[1], data[2], data[3]).
  addManeuverToFlightPlan(mnv).
  local result is mnv:orbit:eccentricity.
  removeManeuverFromFlightPlan(mnv).
  return result.
}

function improve {
  parameter data.
  local scoreToBeat is score(data).
  local bestCandidate is data.
  local candidates is list(
    list(data[0] + 1, data[1], data[2], data[3]),
    list(data[0] - 1, data[1], data[2], data[3]),
    list(data[0], data[1] + 1, data[2], data[3]),
    list(data[0], data[1] - 1, data[2], data[3]),
    list(data[0], data[1], data[2] + 1, data[3]),
    list(data[0], data[1], data[2] - 1, data[3]),
    list(data[0], data[1], data[2], data[3] + 1),
    list(data[0], data[1], data[2], data[3] - 1)
  ).
  for candidate in candidates {
    local candidateScore is score(candidate).
    if candidateScore < scoreToBeat {
      set scoreToBeat to candidateScore.
      set bestCandidate to candidate.
    }
  }
  return bestCandidate.
}

function executeManeuver {
  parameter mList, vessel.
  local mnv is node(mList[0], mList[1], mList[2], mList[3]).
  addManeuverToFlightPlan(mnv).
  local startTime is calculateStartTime(mnv, vessel).
  wait until time:seconds > startTime - 10.
  lockSteeringAtManeuverTarget(mnv).
  wait until time:seconds > startTime.
  // lock throttle to 1.
  lock throttle to max(min(mnv:burnvector:mag / (ship:availablethrust / ship:mass),1),0.005).
  // because this occurs in a preserved function, other preserved functions
  // (such as checking staging) seem to be suspended, so we need to specifically
  // check this one each tick
  until isManeuverComplete(mnv) {
    if stageNeeded(SHIP) {
      doSafeStage().
    }
    wait 0.001.
  }
  lock throttle to 0.
  unlock steering.
  removeManeuverFromFlightPlan(mnv).
}

function addManeuverToFlightPlan {
  parameter mnv.
  add mnv.
}

function calculateStartTime {
  parameter mnv, vessel.
  return time:seconds + mnv:eta - maneuverBurnTime(mnv, vessel) / 2.
}

function maneuverBurnTime {
  parameter mnv, vessel.
  local dV is mnv:deltaV:mag.
  local isp is 0.
  local myEngines is 0.
  list engines in myEngines.
  for en in myEngines {
    if en:ignition and not en:flameout {
      set isp to isp + (en:isp * (en:maxThrust / vessel:maxThrust)).
    }
  }
  local mf is vessel:mass / constant:e^(dV / (isp * constant:g0)).
  local fuelFlow is vessel:maxThrust / (isp * constant:g0).
  local t is (vessel:mass - mf) / fuelFlow.

  return t.
}

function lockSteeringAtManeuverTarget {
  parameter mnv.
  lock steering to mnv:burnvector.
}

function isManeuverComplete {
  parameter mnv.
  if not(defined originalVector) or originalVector = -1 {
    declare global originalVector to mnv:burnvector.
  }
  if vang(originalVector, mnv:burnvector) > 90 {
    declare global originalVector to -1.
    return true.
  }
  return false.
}

function removeManeuverFromFlightPlan {
  parameter mnv.
  remove mnv.
}

function ascentThrottle {
  // local thrott is THROTTLE + PID:UPDATE(TIME:SECONDS, MY_Q).
  // if (thrott < 0) {
    // set thrott to 0.
  // } else if (thrott > 1) {
    // set thrott to 1.
  // }
  WAIT 0.001.
  // return thrott.
  return 1.
}

function ascentHeading {
  parameter tgt_incl.
  return tgt_incl.
}

function ascentPitch {
  parameter vessel.
  // local tp is min(89.9,217.86 - 18.679 * ln(vessel:ALTITUDE)).
  // log fit({100,89.9},{150000,0.1}) on https://www.wolframalpha.com/input/
  local tp is min(89.9,-12.2791 * ln(vessel:APOAPSIS * 0.000000661259)).
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
  // STAGE.
}

// have we experienced a drop in available thrust?
function stageNeeded {
  parameter vessel.
  local t is vessel:AVAILABLETHRUST.
  set TICK to TICK + 1.
  print "Tick:" + TICK at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 5).
  print "Last:" + ROUND(LAST_T,1) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 4).
  print "Avail:" + ROUND(t,1) at (TERMINAL:WIDTH - 16,TERMINAL:HEIGHT - 3).
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
  local parameter t,i,Tmin.
  print "# T MINUS".
  from {local c is t.} until c = 0 step {set c to c - 1.} do {
    print "# ... " + c.
    if (c = i) {
      print "# IGNITION".
      lock throttle to Tmin.
      stage.
    }
    WAIT 1.
  }
}
