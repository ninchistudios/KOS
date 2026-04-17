// THIS SCRIPT SHOULD HAVE NO OUTBOUND DEPENDENCIES
// AND SHOULD BE RUN IN EVERY BOOT FILE BEFORE THE MAIN SCRIPT

@LAZYGLOBAL OFF.

// GLOBALS - use sparingly
global LOGERROR is 3. // Error log message - see Utils.logConsole
global LOGMAJOR is 2. // Major log message - see Utils.logConsole
global LOGADVISORY is 1. // advisory log message - see Utils.logConsole
global LOGTELEMETRY is 0. // advisory log message - see Utils.logConsole

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
local BLANK_TELEMETRY is "                    ". // 20 char

// TODO
function decimalDegrees {
  parameter degs,mins,secs,compass.
  local dd is degs + (mins / 60) + (secs / 3600).
  if compass = "S" {
    set dd to -1 * dd.
  } else if compass = "W" {
    set dd to 360 - dd.
  }
  return dd.
}

// wrapper for logConsole, simplified for regular messages
// param mType : the type of message. 1=advisory, 2=major, 3=error
// param msg : the message. if telemetry, used as the name value (advise max 8 characters for telemetry)
function logMessage {
  parameter mType,msg.
  logConsole(mType,msg,0,0).
}

// utility for neat and consistent console output
// param mType : the type of message. 0=telemetry, 1=advisory, 2=major, 3=error
// param msg : the message. if telemetry, used as the name value (advise max 8 characters for telemetry)
// param val : the telemetry value (max 8 characters). use 0 for other messages
// param index : the line on which the telemetry should be output. TODO refactor as brittle
function logConsole {
  parameter mType,msg,val,index.
  local out is "".
  if mType = LOGTELEMETRY {
    set out to msg + ":" + val.
    print BLANK_TELEMETRY at (TERMINAL:WIDTH - 21, TERMINAL:HEIGHT - index).
    print out at (TERMINAL:WIDTH - (11 + msg:length),TERMINAL:HEIGHT - index).
  } else {
    if mType = LOGADVISORY {
      set out to "# " + msg.
    } else if mType = LOGMAJOR {
      set out to "### " + msg + " ###".
    } else if mType = LOGERROR {
      set out to "#X# ERROR: " + msg + " #X#".
    }
    print out.
  }
}

function neutraliseRoll {
  SET STEERINGMANAGER:ROLLPID:KP TO 0. // set roll rate to 0.
  SET STEERINGMANAGER:ROLLPID:KI TO 0. // set roll rate to 0.
}

function logpid {
  return "P:" + CPp + " I:" + CPi + " D:" + CPd + " +C:" + CLAMP_POS_PITCH + " -C:" + CLAMP_NEG_PITCH.
}

function towerThrottle {
  return 1.
}

function hoverSteering {
  return up.
}

// returns the true radalt of the base of the vehicle, adjusted for height of the control module
// TODO this requires a tweak based on the deployment of landing gear
// param bareRadAlt : the bare radalt without landing struts or launch platforms
// returns the radalt of the vehicle base
function baseRadalt {
  parameter bareRadAlt.
  return MAX(0.00001,((ALTITUDE-GEOPOSITION:TERRAINHEIGHT)-bareRadAlt)).
}

// control the throttle to reduce Q if needed
// TODO implement
// param Tmin: min throttle
// param Tmax: max throttle
// param QMax: limit Q to this if poss
// param iQ: current Q
// param limitQ: should Q be limited
// returns the throttle required to keep Q under Qmax
function ascentThrottle {
  parameter Tmin, Tmax, qMax, iQ, limitQ.
  return Tmax.
}

// TODO
function ascentHeading {
  parameter tgt_incl.
  return tgt_incl.
}

// ascent on a logarithmic path
// TODO take in a max AoA to minimise aero RUDs
// param vess : the vessel being controlled
// returns tp : the instantaneous target pitch to achieve the ascent profile
// Curve fitted for RSS Earth ascent to ~200 km circular orbit.
// Approximate profile: ~89° at apo<10km, ~69° at 20km, ~42° at 50km, ~27° at 80km, ~15° at 120km, ~7° at 160km, 0° at 200km.
// To retarget: set C = 1 / target_apoapsis_in_metres (e.g. 0.000004 for 250km).
// Ref: log fit({10000,89.9},{200000,0}), A=30, C=0.000005
// Previous Kerbin curve: -12.2791 * ln(vessel:APOAPSIS * 0.000000661259)
function ascentPitch {
  parameter vess.
  if vess:APOAPSIS <= 0 { return 89.9. }
  local tp is min(89.9,-30 * ln(vess:APOAPSIS * 0.000005)). // RSS/RO - log fit for 200km target
  return max(0, tp).
}

// vacuum accel-safe deployments (e.g. fairings) should be set to AG8
function deployAccelSafe {
  AG8 ON.
  logMessage(LOGADVISORY,"ACCEL-SAFE MODULES DEPLOYED").
}

// vacuum orbit-safe deployments (e.g. science) should be set to AG9
function deployOrbitSafe {
  AG9 ON.
  logMessage(LOGADVISORY,"ORBIT-SAFE MODULES DEPLOYED").
}

// stage with a delay until ready
function doSafeStage {
  wait until STAGE:READY.
  STAGE.
}

// when updated with the latest Q, returns the highest Q reported
// param latestQ : the currently measured Q
function topQ {
  parameter latestQ.
  if latestQ > HIGHEST_Q {
    set HIGHEST_Q to latestQ.
  }
  return HIGHEST_Q.
}

// usually used to give ullage boosters a chance to do their thing
// param delay : wait at least this number of seconds before staging
function doStageDelay {
  parameter delay.
  set NO_STAGE_BEFORE to TIME:SECONDS + delay.
  set LAST_T to 0.
}

// have we experienced a drop in available thrust, and thus staging needed?
// potentially problematic with multi-mode engines (e.g. on F9)
// parameter vess : the vessel to be checked
// returns bool : is a stage needed
function stageNeeded {
  parameter vess.
  if TIME:SECONDS < NO_STAGE_BEFORE {
    return false.
  }
  local t is vess:AVAILABLETHRUST.
  set TICK to TICK + 1.
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
// param Tmin - min throttle at ignition
// param gt - Gantry stage at T-Minus gt - -1 to disable
function doCountdown {
  local parameter t,i,Tmin,gt.
  doCountdownWithThrottle(t,i,Tmin,gt,-1).
}

// T-minus countdown with optional full-throttle handoff point
// param t - T-Minus count starts at t
// param i - Ignition at T-Minus i - -1 to disable
// param Tmin - min throttle at ignition
// param gt - Gantry stage at T-Minus gt - -1 to disable
// param tf - set throttle to full at T-Minus tf - -1 to disable
function doCountdownWithThrottle {
  local parameter t,i,Tmin,gt,tf.
  local throttlePrimed is false.
  logMessage(LOGADVISORY,"T MINUS").
  from {local c is t.} until c = -1 step {set c to c - 1.} do {
    WAIT 1.
    logMessage(LOGADVISORY," ... " + c).
    if (i <> -1 and not throttlePrimed and c = (i + 1)) {
      lock throttle to Tmin.
      logMessage(LOGADVISORY,"THROTTLE PRIMED TO " + Tmin).
      set throttlePrimed to true.
    }
    if (c = gt) {
      logMessage(LOGADVISORY,"GANTRY").
      doSafeStage().
    }
    if (c = i) {
      logMessage(LOGADVISORY,"IGNITION").
      lock throttle to Tmin.
      doSafeStage().
    }
    if (c = tf and (i = -1 or c < i)) {
      logMessage(LOGADVISORY,"FULL THROTTLE").
      lock throttle to 1.
    }
  }
}

// TODO
//
function doGridfins {
  parameter turnOn.
  if (turnOn) {
    AG3 on.
    logMessage(LOGADVISORY,"GRIDFINS OUT").
  } else {
    AG3 off.
    logMessage(LOGADVISORY,"GRIDFINS IN").
  }
}

function currentMETSeconds {
  return ROUND(TIME:SECONDS - START_TIME, 1).
}

function currentMETFormatDHMS {
  local met is currentMETSeconds().
  local d is FLOOR(met / 86400).
  local h is FLOOR((met - (d * 86400)) / 3600).
  local m is FLOOR((met - (d * 86400) - (h * 3600)) / 60).
  local s is ROUND(met - (d * 86400) - (h * 3600) - (m * 60),1).
  return d + "d " + h + "h " + m + "m " + s + "s".
}

function getResourceAmount {
  parameter resourceName.
  local resList is list().
  list resources in resList.
  for res in resList {
    if res:NAME = resourceName {
      return res:AMOUNT.
    }
  }
  return 0.
}
