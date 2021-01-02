@LAZYGLOBAL OFF.

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
