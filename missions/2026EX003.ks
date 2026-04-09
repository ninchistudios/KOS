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
print "# MISSION: 2026EX003                                     #".
print "# Contract: Suborbital & Return (Uncrewed)               #".
print "#                                                        #".
print "# Mission Objective:                                     #".
print "# - min 140km altitude                                   #".
print "# - Suborbital trajectory                                #".
print "# - return safely                                        #".
print "# - don't RUD                                            #".
print "##########################################################".
print " ".
// CONFIGURE FLIGHT
// Do not include mission specifics in comments, these will be reused and are self-evident
local AGL_BARE is 17.7. // AGL of the bare vehicle at launch - adjust to match your rocket
local CUT_APO is 210000. // MECO if we exceed this apoapsis
local TGT_HEADING is 90. // compass heading after launch
local TGT_PITCH is 89.9. // pitch above horizon during ascent phase (90=vertical, lower=more downrange)
local PITCH_START_ALT is 1000. // altitude at which to begin pitch program
local DESCENT_START_ALT is 140000. // apply descent attitude below this altitude
local CHUTE_STAGE_ALT is 14000. // stage chutes after aerobraking
local DESCENT_PITCH is 90. // descent pitch above horizon
local DO_WARP is false. // set true to physics warp through boring bits
local WARP_SPEED is 3. // 1/2/3 corresponding to 2x/3x/4x physics warp
local TCOUNT is 7. // Tminus countdown start
local TIGNITE is 5. // Ignition at Tminus (at Tmin throttle)
local TTHROTTLE is 1. // Tminus for full throttle
local TGANTRY is 0. // Gantry/clamp release at Tminus
local Tmin is 0.32. // minimum throttle at ignition
local TELEMETRY_ENABLED is true. // log to console
local LOGGING_ENABLED is true. // log to CSV
local DEPLOY_ACCEL_SAFE is true. // deploy AG8 modules once atmo drag is no longer a factor
local DEPLOY_ACCEL_SAFE_ALT is 100000. // min altitude for AG8 deployment
local DEPLOY_ACCEL_SAFE_Q is 0. // max Q (kPa) for AG8 deployment
// Mission parameters for success tracking
local ENGINE_NOMINAL_BURN is 83.0. // seconds of nominal burn time for the main engine - calibrate during testflights
local MISSION_MIN_APO is 140000. // apoapsis
local MISSION_MIN_AVIONICSTIME is 50. // minimum avionics duration in seconds
local MISSION_GOAL_SAFELANDING is false.
local MISSION_GOAL_MINAPO is false.
local MISSION_GOAL_SUBORBITAL is true.
local MISSION_GOAL_AVIONICSTIME is false.
// END CONFIGURE FLIGHT

// CONSTANTS AND GLOBALS
local LAUNCH_AMSL is ROUND(ship:ALTITUDE,3).
local LAUNCH_AGL is ROUND(MAX(0.001,(ALTITUDE-GEOPOSITION:TERRAINHEIGHT)),3).
local LAUNCH_GEO is SHIP:GEOPOSITION. // saved for downrange distance calculation
local MY_VESSEL is SHIP.
lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa.
lock TOP_Q to topQ(MY_Q).
lock DOWNRANGE to MY_VESSEL:BODY:RADIUS * VANG(
  // great-circle surface distance from launch site in meters
  LAUNCH_GEO:POSITION - MY_VESSEL:body:position,
  MY_VESSEL:position - MY_VESSEL:body:position
) * constant:degtorad.
global START_TIME is TIME:SECONDS.
local LOGGED_PITCH is 0.
local NEXT_LOG_TIME is TIME:SECONDS + 1.
local AUTOPILOT is true. // program will run until this is switched off
local ACCEL_SAFE_DEPLOYED is false.
local AGL is 0.
local AVIONICS_TIME_ALERTED is false.
local MAX_PERIAPSIS is 0.
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
    doMissionGoals().
    doTowerPhase().

    // clear of the tower, begin pitch program
    when AGL > (2 * LAUNCH_AGL) THEN {
      logMessage(LOGADVISORY,"TOWER CLEAR").
      doAscentPhase().

      // burnout or risking unplanned orbit - cut engines
      when (stageNeeded(MY_VESSEL) or MY_VESSEL:periapsis > 0 or MY_VESSEL:APOAPSIS > CUT_APO) then {
        doMECO().

        // ballistic coast - wait for impact
        when (MY_VESSEL:status = "LANDED" or MY_VESSEL:status = "SPLASHED") then {
          doTouchDown().
        }
      }
    }
  }
}

function doMissionGoals {

  when (MY_VESSEL:status = "LANDED" or MY_VESSEL:status = "SPLASHED") THEN {
    if (MY_VESSEL:VERTICALSPEED < 6) {
      set MISSION_GOAL_SAFELANDING to true.
      logMessage(LOGADVISORY,"Mission Goal Achieved: Safe Landing").
    }
  }
  
  when MY_VESSEL:altitude >= MISSION_MIN_APO THEN {
    set MISSION_GOAL_MINAPO to true.
    logMessage(LOGADVISORY,"Mission Goal Achieved: Min Apoapsis").
  }
  
  when MY_VESSEL:PERIAPSIS > MAX_PERIAPSIS THEN {
    set MAX_PERIAPSIS to MY_VESSEL:PERIAPSIS.
  }
  
  when MAX_PERIAPSIS > 140000 THEN {
    set MISSION_GOAL_SUBORBITAL to false.
    logMessage(LOGADVISORY,"Mission Goal Failed: Not Suborbital").
  }

  when currentMETseconds() > MISSION_MIN_AVIONICSTIME THEN {
    set MISSION_GOAL_AVIONICSTIME to true.
    logMessage(LOGADVISORY,"Mission Goal Achieved: Min Avionics Time").
  }

}

function doTowerPhase {
  logMessage(LOGMAJOR,"TOWER PHASE").
  logMessage(LOGADVISORY, "Target Heading: " + TGT_HEADING + " / Ascent Pitch: " + TGT_PITCH).
  logMessage(LOGADVISORY, "Pitch Program Alt: " + PITCH_START_ALT/1000 + "km").
  switch to archive.
  if (LOGGING_ENABLED) {
    log logpid() + ",,,," to MYLOGFILE.
    log "MET,ALT,PITCH,Q,APO,DOWNRANGE" to MYLOGFILE.
  }
  doCountdownWithThrottle(TCOUNT, TIGNITE, Tmin, TGANTRY, TTHROTTLE).
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
  // deploy fairings and similar AG8 payloads as soon as aero risk is effectively gone
  // can also stage the avionics away from the engines to reduce chute needs
  when (MY_VESSEL:ALTITUDE >= DEPLOY_ACCEL_SAFE_ALT and MY_Q <= DEPLOY_ACCEL_SAFE_Q and DEPLOY_ACCEL_SAFE and not ACCEL_SAFE_DEPLOYED) then {
      deployAccelSafe().
      set ACCEL_SAFE_DEPLOYED to true.
    }

  // alert through max Q
  when TOP_Q > (MY_Q + 1) THEN {
    logMessage(LOGADVISORY,"THROUGH MAX Q " + ROUND(TOP_Q,1) + " KPA AT " + ROUND(MY_VESSEL:ALTITUDE / 1000,1) + " KM").
  }

  // vertical until PITCH_START_ALT, then pitch to target heading
  when MY_VESSEL:ALTITUDE >= PITCH_START_ALT THEN {
    logMessage(LOGADVISORY,"Pitch Program - HDG " + TGT_HEADING + " Pitch " + TGT_PITCH).
    lock steering to heading(TGT_HEADING, TGT_PITCH).
  }

  when getResourceAmount("ELECTRICCHARGE") < 0.1 THEN {
    logMessage(LOGERROR,"LOW ELECTRICITY").
  }
}

function doMECO {
  logMessage(LOGMAJOR,"MECO").
  if DO_WARP { set warp to 0. }
  lock throttle to 0.
  local met is ROUND(TIME:SECONDS - START_TIME, 1).
  logMessage(LOGADVISORY,"BURNOUT MET+" + met + "s / DOWNRANGE " + ROUND(DOWNRANGE/1000,1) + "km").
  if (met < ENGINE_NOMINAL_BURN or MY_VESSEL:APOAPSIS < MISSION_MIN_APO) {
    logMessage(LOGERROR,"ENGINE OUT OF NOMINAL").
  } else {
    logMessage(LOGADVISORY,"ENGINE BURN NOMINAL").
  }
  
  lock steering to up.
  logMessage(LOGADVISORY,"BALLISTIC").
  
  when (MY_VESSEL:ALTITUDE >= DEPLOY_ACCEL_SAFE_ALT and MY_Q <= DEPLOY_ACCEL_SAFE_Q) then {
    logMessage(LOGADVISORY,"STAGING AVIONICS AND PAYLOAD").
    doSafeStage().
  }
  
  when (MY_VESSEL:ALTITUDE <= CHUTE_STAGE_ALT and MY_VESSEL:VERTICALSPEED < 0) then {
    logMessage(LOGADVISORY,"CHUTE DEPLOYMENT AT " + ROUND(CHUTE_STAGE_ALT/1000,1) + "KM").
    // deploy chutes here - set them to open at a safe altitude and speed for the landing
    doSafeStage().
    when (MY_VESSEL:verticalspeed < 0 and MY_VESSEL:verticalspeed > -6) then {
      logMessage(LOGADVISORY,"GOOD CHUTE").
    }
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
  logMessage(LOGMAJOR, "FINAL MET: " + currentMETFormatDHMS()).
  logMessage(LOGMAJOR, "MISSION GOALS:").
  logMessage(LOGMAJOR, " - Min Apoapsis: " + MISSION_GOAL_MINAPO).
  logMessage(LOGMAJOR, " - Suborbital: " + MISSION_GOAL_SUBORBITAL).
  logMessage(LOGMAJOR, " - Safe Landing: " + MISSION_GOAL_SAFELANDING).
  logMessage(LOGMAJOR, " - Min Avionics Time: " + MISSION_GOAL_AVIONICSTIME).
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
  to MYLOGFILE.
}
