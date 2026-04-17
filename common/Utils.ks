// THIS SCRIPT SHOULD HAVE NO OUTBOUND DEPENDENCIES
// AND SHOULD BE RUN IN EVERY BOOT FILE BEFORE THE MAIN SCRIPT

@LAZYGLOBAL OFF.

// GLOBALS - use sparingly
global LOGERROR is 3. // Error log message - see Utils.logConsole
global LOGMAJOR is 2. // Major log message - see Utils.logConsole
global LOGADVISORY is 1. // advisory log message - see Utils.logConsole
global LOGTELEMETRY is 0. // advisory log message - see Utils.logConsole

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
