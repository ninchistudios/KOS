// Standard AGs (DO NOT REMOVE):
// AG1: Engine mode (F9 9/3/1 SH 28/7)
// AG2: Engine Mode 2 (e.g. VTOL)
// AG3: Grid Fins toggle
// AG7: PV Panels (atmo/accel unsafe)
// AG8: Vacuum Accel-safe Modules (e.g. fairings) - safe to do a burn after deployed
// AG9: Vacuum Accel-risk Modules (e.g. big antennas) - only deployed when there are no more burns
// AG0: KAL9000 power toggle

@LAZYGLOBAL OFF.

print " ".
print "##########################################################".
print "# MISSION: " + MISSION_ID + "                                     #".
print "# EX Downrange Distance LV                               #".
print "#                                                        #".
print "# Mission Objective:                                     #".
print "# - 150km downrange with 500u sounding payload           #".
print "# - min 50s of avionics control                          #".
print "# - min 140km altitude                                   #".
print "# - don't RUD                                            #".
print "##########################################################".
print " ".
// CONFIGURE FLIGHT
local AGL_BARE is 16.0. // AGL of the bare vehicle at launch - adjust to match your rocket
local TGT_APO is 145000. // target apoapsis - min 140km per contract
local TGT_HEADING is 270. // compass heading toward downrange target
local TGT_PITCH is 70. // pitch above horizon during ascent phase (90=vertical, lower=more downrange)
local PITCH_START_ALT is 1000. // altitude at which to begin pitch program
local DESCENT_START_ALT is 140000. // apply descent attitude below this altitude (top of effective atmosphere)
local DESCENT_PITCH is -75. // descent pitch above horizon
local MIN_AVIONICS_TIME is 55. // minimum avionics duration in seconds (contract requires 50s; 55s adds margin)
local DO_WARP is false. // set true to physics warp through boring bits
local WARP_SPEED is 3. // 1/2/3 corresponding to 2x/3x/4x physics warp
local TCOUNT is 7. // T-Minus countdown
local TGANTRY is 0. // Gantry/clamp release at T-
local TIGNITE is 5. // Ignition at T-
local Tmin is 0.1. // minimum throttle at ignition
local TELEMETRY_ENABLED is true. // log to console
local LOGGING_ENABLED is true. // log to CSV
// END CONFIGURE FLIGHT

// CONSTANTS AND GLOBALS
local LAUNCH_AMSL is ROUND(ship:ALTITUDE,3).
local LAUNCH_AGL is ROUND(MAX(0.001,(ALTITUDE-GEOPOSITION:TERRAINHEIGHT)),3).
local LAUNCH_GEO is SHIP:GEOPOSITION. // saved for downrange distance calculation
local MY_VESSEL is SHIP.
lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa.
lock TOP_Q to topQ(MY_Q).
// great-circle surface distance from launch site in metres
lock DOWNRANGE to MY_VESSEL:BODY:RADIUS * VANG(
  LAUNCH_GEO:POSITION - MY_VESSEL:body:position,
  MY_VESSEL:position - MY_VESSEL:body:position
) * constant:degtorad.
local START_TIME to TIME:SECONDS.
local LOGGED_PITCH is 0.
local NEXT_LOG_TIME is TIME:SECONDS + 1.
local AUTOPILOT is true. // program will run until this is switched off
local AGL is 0.
if archive:exists("TestFlight.csv") {
  archive:delete("TestFlight.csv").
}
local MYLOGFILE to archive:create("TestFlight.csv").

// RUN
doSetup(). // runs once
doMain(). // runs continuously until exited
doFinalise(). // runs once
// -------------------------------
// FUNCTIONS ONLY BELOW THIS POINT

// the main sequence of the flight plan
function doFlightTriggers {

  when true THEN {
    doTowerPhase().

    // clear of the tower, begin pitch program
    when AGL > (2 * LAUNCH_AGL) THEN {
      logMessage(LOGADVISORY,"TOWER CLEAR").
      doAscentPhase().

      // burnout or target apoapsis reached - cut engines
      when (stageNeeded(MY_VESSEL) or MY_VESSEL:periapsis > 0) then {
        doMECO().

        // ballistic coast - wait for impact
        when (MY_VESSEL:status = "LANDED" or MY_VESSEL:status = "SPLASHED") then {
          doTouchDown().
        }
      }
    }
  }
}

function doTowerPhase {
  logMessage(LOGMAJOR,"TOWER PHASE").
  logMessage(LOGADVISORY, "Target Apoapsis: " + TGT_APO/1000 + "km").
  logMessage(LOGADVISORY, "Target Heading: " + TGT_HEADING + " / Ascent Pitch: " + TGT_PITCH).
  logMessage(LOGADVISORY, "Pitch Program Alt: " + PITCH_START_ALT/1000 + "km").
  logMessage(LOGADVISORY, "Min Avionics: " + MIN_AVIONICS_TIME + "s").
  switch to archive.
  if (LOGGING_ENABLED) {
    log logpid() + ",,,," to MYLOGFILE.
    log "MET,ALT,PITCH,Q,APO,DOWNRANGE" to MYLOGFILE.
  }
  doCountdown(TCOUNT, TIGNITE, Tmin, TGANTRY).
  lock throttle to towerThrottle().
  logMessage(LOGADVISORY,"LIFTOFF").
  lock steering to up.
  wait 1.
  rcs off.
}

function doAscentPhase {
  logMessage(LOGMAJOR,"ASCENT PHASE").
  if DO_WARP { set warp to WARP_SPEED. }
  lock throttle to 1.
  rcs off.
  // alert through max Q
  when TOP_Q > (MY_Q + 1) THEN {
    logMessage(LOGADVISORY,"THROUGH MAX Q " + ROUND(TOP_Q,1) + " KPA AT " + ROUND(MY_VESSEL:ALTITUDE / 1000,1) + " KM").
  }
  // vertical until PITCH_START_ALT, then pitch to target heading
  when MY_VESSEL:ALTITUDE >= PITCH_START_ALT THEN {
    logMessage(LOGADVISORY,"PITCH PROGRAM - HDG " + TGT_HEADING + " PITCH " + TGT_PITCH).
    lock steering to heading(TGT_HEADING, TGT_PITCH).
  }
}

function doMECO {
  logMessage(LOGMAJOR,"MECO").
  if DO_WARP { set warp to 0. }
  lock throttle to 0.
  local met is ROUND(TIME:SECONDS - START_TIME, 1).
  logMessage(LOGADVISORY,"BURNOUT MET+" + met + "s / DOWNRANGE " + ROUND(DOWNRANGE/1000,1) + "km").
  if met < MIN_AVIONICS_TIME {
    logMessage(LOGERROR,"AVIONICS TIME " + met + "s BELOW CONTRACT MIN " + MIN_AVIONICS_TIME + "s").
  } else {
    logMessage(LOGADVISORY,"AVIONICS " + met + "s - OK").
  }
  lock steering to up.
  logMessage(LOGADVISORY,"BALLISTIC COAST").
  // once descending and back under top-of-atmo, hold configured downrange attitude
  when (MY_VESSEL:VERTICALSPEED < 0 and MY_VESSEL:ALTITUDE <= DESCENT_START_ALT) then {
    logMessage(LOGADVISORY,"DESCENT ATTITUDE - HDG " + TGT_HEADING + " PITCH " + DESCENT_PITCH + " BELOW " + ROUND(DESCENT_START_ALT/1000,0) + "KM").
    lock steering to heading(TGT_HEADING, DESCENT_PITCH).
  }
}

function doTouchDown {
  logMessage(LOGMAJOR,"SPLASHDOWN / TOUCHDOWN").
  logMessage(LOGADVISORY,"FINAL DOWNRANGE: " + ROUND(DOWNRANGE/1000,1) + "km").
  logMessage(LOGADVISORY,"MAX APO: " + ROUND(MY_VESSEL:APOAPSIS/1000,1) + "km").
  lock throttle to 0.
  set MY_VESSEL:control:neutralize to true.
  set AUTOPILOT to false.
  logMessage(LOGADVISORY,"GUIDANCE OFFLINE - OK").
}

// used for triggers that can run multiple times
// note that preserve has been replaced by return - https://ksp-kos.github.io/KOS/language/flow.html
function doPreservedTriggers {

}

function doTelemetry {
  if TELEMETRY_ENABLED {
    // logConsole parameters mType,msg,val,index
    logConsole(LOGTELEMETRY,"Q",ROUND(MY_Q,1),6).
    logConsole(LOGTELEMETRY,"Mass",ROUND(MY_VESSEL:mass,1),5).
    logConsole(LOGTELEMETRY,"Vv",ROUND(MY_VESSEL:verticalspeed,1),4).
    logConsole(LOGTELEMETRY,"APO",ROUND(MY_VESSEL:APOAPSIS/1000,1),3).
    logConsole(LOGTELEMETRY,"ALT",ROUND(MY_VESSEL:ALTITUDE/1000,1),2).
    logConsole(LOGTELEMETRY,"DWNRNG",ROUND(DOWNRANGE/1000,1),1).
  }
}

// runs once at the start of the script
function doSetup {
  neutraliseRoll().
  lock AGL to baseRadalt(AGL_BARE).
  set kuniverse:TimeWarp:MODE to "PHYSICS".
  logMessage(LOGADVISORY,"Launch AMSL: " + LAUNCH_AMSL + "m").
  logMessage(LOGADVISORY,"Launch AGL: " + LAUNCH_AGL + "m").
  logMessage(LOGADVISORY,"Hardware AGL: " + AGL_BARE + "m").
  logMessage(LOGADVISORY,"Wet Mass: " + ROUND(MY_VESSEL:wetmass,1) + "T").
  logMessage(LOGADVISORY,"Dry Mass: " + ROUND(MY_VESSEL:drymass,1) + "T").
}

// loops while inflight
function doMain {
  doPreservedTriggers().
  doFlightTriggers().
  until not AUTOPILOT {
    doTelemetry().
    if (LOGGING_ENABLED) {
      if TIME:SECONDS >= NEXT_LOG_TIME {
        set NEXT_LOG_TIME to NEXT_LOG_TIME + 1.
        logTelemetry().
      }
    }
    WAIT 0.
  }
}

// run last
function doFinalise {
  lock THROTTLE to 0.
  set MY_VESSEL:control:neutralize to true.
  logMessage(LOGMAJOR,"PROGRAM COMPLETE").
  until false {
    wait 1.
  }
}

function logTelemetry {
  // log pitch above local horizon (90 = straight up, 0 = horizon)
  set LOGGED_PITCH to 90 - VANG(MY_VESSEL:FACING:FOREVECTOR, MY_VESSEL:UP:FOREVECTOR).
  log
    (TIME:SECONDS - START_TIME)
    + ","
    + MY_VESSEL:ALTITUDE
    + ","
    + LOGGED_PITCH
    + ","
    + MY_Q
    + ","
    + MY_VESSEL:APOAPSIS
    + ","
    + DOWNRANGE
  to "TestFlight.csv".
}
