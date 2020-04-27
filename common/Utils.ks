@LAZYGLOBAL OFF.

local LAST_T is 0.
local TICK is 0.
local HIGHEST_Q is 0.
local CIRC_PITCH is 5.
local CPp is 0.04.
local CPi is 0.01.
local CPd is 0.15.
local CPPID is PIDLOOP(CPp,CPi,CPd,-0.1,0.1).
local PRE_APO is true.
local NO_STAGE_BEFORE is 0.
local CLAMP_POS_PITCH is 15.
local CLAMP_NEG_PITCH is -30.

function logpid {
  return "P:" + CPp + " I:" + CPi + " D:" + CPd + " +C:" + CLAMP_POS_PITCH + " -C:" + CLAMP_NEG_PITCH.
}

function towerThrottle {
  return 1.
}

function hoverSteering {
  return up.
}

// note this changes with fuel burn, so... TODO need to figure this out.
function baseRadalt {
  parameter launchRadAlt.
  return MAX(0.00001,((ALTITUDE-GEOPOSITION:TERRAINHEIGHT)-launchRadAlt)).
}

// thanks CheersKevin
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

// thanks CheersKevin
function score {
  parameter data.
  local mnv is node(data[0], data[1], data[2], data[3]).
  addManeuverToFlightPlan(mnv).
  local result is mnv:orbit:eccentricity.
  removeManeuverFromFlightPlan(mnv).
  return result.
}

// thanks CheersKevin
function improve {
  parameter data.
  local scoreToBeat is score(data).
  local bestCandidate is data.
  local tfactor is 10.
  local ofactor is 1.
  local dfactor is 100.
  local candidates is list(
    list(data[0] + tfactor, data[1], data[2], data[3]),
    list(data[0] - tfactor, data[1], data[2], data[3]),
    list(data[0], data[1] + ofactor, data[2], data[3]),
    list(data[0], data[1] - ofactor, data[2], data[3]),
    list(data[0], data[1], data[2] + ofactor, data[3]),
    list(data[0], data[1], data[2] - ofactor, data[3]),
    list(data[0], data[1], data[2], data[3] + dfactor),
    list(data[0], data[1], data[2], data[3] - dfactor)
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

// thanks CheersKevin
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
  // until isManeuverComplete(mnv) {
    // if stageNeeded(SHIP) {
    //   doSafeStage().
  //   }
  // }
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

// control the throttle to reduce Q if needed
// param Tmin: min throttle
// param Tmax: max throttle
// param QMax: limit Q to this if poss
// param iQ: current Q
// param limitQ: should Q be limited
function ascentThrottle {
  parameter Tmin, Tmax, qMax, iQ, limitQ.
  // print "THROTT:" + ROUND(THROTTLE,1) at (TERMINAL:WIDTH - 17,TERMINAL:HEIGHT - 4).
  return Tmax.
}

function ascentHeading {
  parameter tgt_incl.
  return tgt_incl.
}

// ascent on a logarithmic path
function ascentPitch {
  parameter vessel.
  // local tp is min(89.9,217.86 - 18.679 * ln(vessel:ALTITUDE)).
  // log fit({100,89.9},{150000,0.1}) on https://www.wolframalpha.com/input/
  local tp is min(89.9,-12.2791 * ln(vessel:APOAPSIS * 0.000000661259)).
  print "APTCH:" + ROUND(tp,1) + "  " at (TERMINAL:WIDTH - 16,4).
  return tp.
}

function initCircPID {
  parameter tgtAPO.
  set CPPID:SETPOINT to tgtAPO.
}

// circularisation, adjusting pitch to keep APO at TGT_APO
function circPitch {
  parameter vessel,flip.
  if (flip) {
    // past APO, invert pitch
    set CIRC_PITCH to max(CLAMP_NEG_PITCH,min(CLAMP_POS_PITCH,CIRC_PITCH - CPPID:UPDATE(TIME:seconds,vessel:APOAPSIS))).
  } else {
      set CIRC_PITCH to max(CLAMP_NEG_PITCH,min(CLAMP_POS_PITCH,CIRC_PITCH + CPPID:UPDATE(TIME:seconds,vessel:APOAPSIS))).
  }
  print "CPTCH:" + ROUND(CIRC_PITCH,1) + "  " at (TERMINAL:WIDTH - 16,5).
  return CIRC_PITCH.
}

// vacuum accel-safe deployments (e.g. fairings) should be set to AG1
function deployAccelSafe {
  AG1 ON.
  print "# ACCEL-SAFE MODULES DEPLOYED #".
}

// vacuum orbit-safe deployments (e.g. science) should be set to AG2
function deployOrbitSafe {
  AG2 ON.
  print "# ORBIT-SAFE MODULES DEPLOYED #".
}

// stage with a delay until ready
function doSafeStage {
  wait until STAGE:READY.
  // print "# STAGING #".
  STAGE.
}

function topQ {
  parameter latestQ.
  if latestQ > HIGHEST_Q {
    set HIGHEST_Q to latestQ.
  }
  return HIGHEST_Q.
}

// usually used to give ullage boosters a chance to burn out
function doStageDelay {
  parameter delay.
  set NO_STAGE_BEFORE to TIME:SECONDS + delay.
  set LAST_T to 0.
}

// have we experienced a drop in available thrust?
function simpleStageNeeded {
  parameter vessel.
  if TIME:SECONDS < NO_STAGE_BEFORE {
    return false.
  }
  local t is vessel:AVAILABLETHRUST.
  set TICK to TICK + 1.
  //print "Tick:" + TICK at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 7).
  //print "Last:" + ROUND(LAST_T,1) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 6).
  //print "Avail:" + ROUND(t,1) at (TERMINAL:WIDTH - 16,TERMINAL:HEIGHT - 5).
  if (t = 0 or t < (LAST_T - 10)) {
    // uh oh, loss of available thrust
    set LAST_T to t.
    return true.
  } else {
    set LAST_T to t.
    return false.
  }
}

// have we experienced a drop in available thrust?
function stageNeeded {
  parameter vessel.

  local t is vessel:AVAILABLETHRUST.
  set TICK to TICK + 1.
  //print "Tick:" + TICK at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 7).
  //print "Last:" + ROUND(LAST_T,1) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 6).
  //print "Avail:" + ROUND(t,1) at (TERMINAL:WIDTH - 16,TERMINAL:HEIGHT - 5).
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
// param t - T-Minus count starts at t
// param i - Ingition at T-Minus i
// Tmin - min throttle at ignition
function doCountdown {
  local parameter t,i,Tmin.
  print "# T MINUS".
  from {local c is t.} until c = 0 step {set c to c - 1.} do {
    print "# ... " + c.
    if (c = i) {
      print "# IGNITION".
      lock throttle to Tmin.
      doSafeStage().
    }
    WAIT 1.
  }
}
