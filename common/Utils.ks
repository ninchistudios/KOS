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
// param launchRadAlt : the reported radalt at launch
// returns the radalt of the vehicle base
function baseRadalt {
  parameter launchRadAlt.
  return MAX(0.00001,((ALTITUDE-GEOPOSITION:TERRAINHEIGHT)-launchRadAlt)).
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
// param vessel : the vessel being controlled
// returns tp : the instantaneous target pitch to achieve the ascent profile
function ascentPitch {
  parameter vessel.
  // local tp is min(89.9,217.86 - 18.679 * ln(vessel:ALTITUDE)).
  // calc on https://www.wolframalpha.com/input/
  local tp is min(89.9,-12.2791 * ln(vessel:APOAPSIS * 0.000000661259)). // steep on Kerbin - log fit({100,89.9},{150000,0.1})
  return tp.
}

// vacuum accel-safe deployments (e.g. fairings) should be set to AG1
function deployAccelSafe {
  AG8 ON.
  logMessage(LOGADVISORY,"ACCEL-SAFE MODULES DEPLOYED").
}

// vacuum orbit-safe deployments (e.g. science) should be set to AG2
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
// parameter vessel : the vessel to be checked
// returns bool : is a stage needed
function stageNeeded {
  parameter vessel.
  if TIME:SECONDS < NO_STAGE_BEFORE {
    return false.
  }
  local t is vessel:AVAILABLETHRUST.
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
  logMessage(LOGADVISORY,"T MINUS").
  from {local c is t.} until c = 0 step {set c to c - 1.} do {
    logMessage(LOGADVISORY," ... " + c).
    if (c = gt) {
      logMessage(LOGADVISORY,"GANTRY").
      doSafeStage().
    }
    if (c = i) {
      logMessage(LOGADVISORY,"IGNITION").
      lock throttle to Tmin.
      doSafeStage().
    }
    WAIT 1.
  }
}

// TODO
//
function doGridfins {
  parameter turnOn.
  if (turnOn) {
    AG7 on.
    logMessage(LOGADVISORY,"GRIDFINS OUT").
  } else {
    AG7 off.
    logMessage(LOGADVISORY,"GRIDFINS IN").
  }
}
