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
local CLAMP_NEG_PITCH is -89.

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

// vacuum accel-safe deployments (e.g. fairings) should be set to AG1
function deployAccelSafe {
  AG8 ON.
  print "# ACCEL-SAFE MODULES DEPLOYED #".
}

// vacuum orbit-safe deployments (e.g. science) should be set to AG2
function deployOrbitSafe {
  AG9 ON.
  print "# ORBIT-SAFE MODULES DEPLOYED #".
}

// stage with a delay until ready
function doSafeStage {
  wait until STAGE:READY.
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
// param i - Ingition at T-Minus i - -1 to disable
// Tmin - min throttle at ignition
// param gt - Gantry stage at T-Minus gt - -1 to disable
function doCountdown {
  local parameter t,i,Tmin,gt.
  print "# T MINUS".
  from {local c is t.} until c = 0 step {set c to c - 1.} do {
    print "# ... " + c.
    if (c = gt) {
      print "# GANTRY".
      doSafeStage().
    }
    if (c = i) {
      print "# IGNITION".
      lock throttle to Tmin.
      doSafeStage().
    }
    WAIT 1.
  }
}
