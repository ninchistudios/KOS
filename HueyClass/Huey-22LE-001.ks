@LAZYGLOBAL OFF.

// include utilities once and only once
runoncepath("Utils").

print " ".
print "################################".
print "# HUEY-22LE-001 FLIGHT PROGRAM #".
print "# 22T TEST MASS LEO INSERTION  #".
print "################################".
print " ".

// AG settings:
// AG1: vacuum accel-safe (e.g. fairings)
// AG2: vacuum orbit-safe (e.g. antennas, experiments etc)
// AG9: used for debug info

// ########## CONFIGURE LAUNCH #############
local ATMO_BOUNDARY is 140000. // where does the atmo end
local TGT_APO is 200000. // target orbit to be circularised
local TGT_PERI is 170000. // target orbit to be circularised
local TGT_ASC_ROLL is 270. // do a special ascent roll program stage
local TGT_INCL is 118.5. // target orbital inclination, Canaveral lowest energy launch = 118.5
local TCOUNT is 5. // T-Minus countdown
local IGNITE_TMINUS IS 3. // Pre-Ignites RealFuels Engines if > 0
local LIMIT_Q is false. // limit Q to terminal velocity? CURRENTLY NONFUNCTIONAL
local MAX_Q is 40. // If limiting Q, limit to what?
local FULL_THROTT_OVER is 22000. // hacky way of limiting Q under this alt
local EST_CIRC_DV is 5000. // estimated circularisation dV.
local STAGE_DISABLED is false. // disables all auto staging
local DEPLOY_ORBIT_SAFE is true. // deploy orbit-safe modules when finalising plan?
local MAX_CIRC_VV is 500. // once in circularisation, the max vertical velocity
local LOGGING_ENABLED is true.
// Stage List Parameters
// a: KSP Stage No
// b: Stage Type:
//    "PRE" - Pre-ignition
//    "TWR" - Tower Release
//    "STS" - Stage Separate
//    "STI" - Stage Ignite
//    "FAI" - Fairing Jettison (above atmo)
//    "MAN" - Manual Stage - pause auto staging
// c: Flight Phase this should occur in:
//    "TO" - Tower
//    "AT" - Atmo Penetration
//    "AS" - Ascent
//    "BA" - Ballistic
//    "MN" - Maneuver
//    "OR" - Orbital
// d: Min Throttle for Stage
// e: Max Throttle for Stage
local StageList is list (
  list(8,"PRE","TO",0.2,0.2),
  list(7,"TWR","TO",0.2,0.5),
  list(6,"SEP","AS",0.2,1.0),
  list(5,"SEP","AS",0.2,1.0),
  list(4,"IGN","AS",0.2,1.0),
  list(3,"SEP","BA",0.2,1.0),
  list(2,"IGN","MN",0.2,1.0),
  list(1,"MAN","OR",0.2,1.0),
  list(0,"MAN","OR",0.2,1.0)
).
// ########## END CONFIGURE LAUNCH #############

// aliases
local MY_VESSEL is SHIP. // safes against vehicle switch
local AUTOPILOT is true. // program will run until this is switched off
local LAUNCH_ALT is ROUND(MY_VESSEL:ALTITUDE,1). // alt of the control module
lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa. // dynamic pressure in kPa
lock TOP_Q to topQ(MY_Q).
local CURR_STAGE to -1.
local PAST_APO to false.
local START_TIME to TIME:SECONDS.
local LOGGED_PITCH is 0.
local NEXT_LOG_TIME is TIME:SECONDS + 1.
if archive:exists("TestFlight.csv") {
  archive:delete("TestFlight.csv").
}
local LOGFILE to archive:create("TestFlight.csv").
// RUN
doSetup().
doMain().
doFinalise().

// runs once
function doSetup {
  // surface key flight data
  print "Target Orbit: " + TGT_APO + "m".
  print "Target Orbital Inclination: " + TGT_INCL + " degrees".
  print "Launch AMSL: " + LAUNCH_ALT + "m".
  print "Limiting to Terminal Velocity: " + LIMIT_Q.
  if (STAGE_DISABLED) {
    print "*** STAGING DISABLED - MANUAL STAGING REQUIRED ***".
  } else {
    print "Automated Staging Enabled".
  }
  print "Logging telemetry: " + LOGGING_ENABLED.
  lock THROTTLE to 0.
  switch to archive.
  if (LOGGING_ENABLED) {
    log logpid() + ",,,,," to "TestFlight.csv".
    log "MET,ALT,PITCH,Q,APO,PERI,TGTAPO,TGTPERI" to "TestFlight.csv".
  }
}

// loops while program executing
function doMain {

  doPreservedTriggers().
  doFlightTriggers().

  until not AUTOPILOT {
    print "    Q:" + ROUND(MY_Q,1) + "  " at (TERMINAL:WIDTH - 16,0).
    print "STAGE:" + CURR_STAGE + "     " at (TERMINAL:WIDTH - 16,1).
    print "  TTA:" + ROUND(ETA:APOAPSIS,0) + "   " at (TERMINAL:WIDTH - 16,2).
    print "  TTP:" + ROUND(ETA:PERIAPSIS,0) + "   " at (TERMINAL:WIDTH - 16,3).
    if (LOGGING_ENABLED) {
      if TIME:SECONDS >= NEXT_LOG_TIME {
        set NEXT_LOG_TIME to NEXT_LOG_TIME + 1.
        doTelemetry().
      }
    }
    WAIT 0.
  }
}

// this is the main sequence of the flight plan
// driven by the StageList
function doFlightTriggers {

  // launch
  when MY_VESSEL:ALTITUDE < 2 * LAUNCH_ALT THEN {
    print "# TOWER PHASE #".
    set CURR_STAGE to CURR_STAGE + 1.
    doCountdown(TCOUNT,IGNITE_TMINUS,StageList[CURR_STAGE][3]).
    set CURR_STAGE to CURR_STAGE + 1.
    lock THROTTLE to ascentThrottle(StageList[CURR_STAGE][3],StageList[CURR_STAGE][4],MAX_Q,MY_Q,LIMIT_Q).
    lock steering to LOOKDIRUP(MY_VESSEL:UP:FOREVECTOR, MY_VESSEL:FACING:TOPVECTOR).
    print "# LAUNCH".
    set START_TIME to TIME:SECONDS.
    doSafeStage().

    // cleared tower
    when MY_VESSEL:ALTITUDE > 2 * LAUNCH_ALT THEN {
      print "# TOWER CLEAR".
      print "# ATMO PENETRATION PHASE #".
      lock steering to heading(ascentHeading(TGT_INCL), ascentPitch(MY_VESSEL),TGT_ASC_ROLL).

      // through the soup, go full burn
      when MY_VESSEL:ALTITUDE > FULL_THROTT_OVER then {
        print "# CLEAR AIR ASCENT PHASE #".
        lock THROTTLE to ascentThrottle(StageList[CURR_STAGE][3],1.0,MAX_Q,MY_Q,false).

        // flatten the ascent
        when MY_Q < 1 then {
          print "# Q1 AT " + ROUND(MY_VESSEL:ALTITUDE / 1000,1) + " KM".
          print "# CONTINUOUS CIRCULARISATION PHASE #".
          initCircPID(TGT_APO).
          lock steering to heading(ascentHeading(TGT_INCL), circPitch(MY_VESSEL,PAST_APO),TGT_ASC_ROLL).
        }

        // pickle the boosters
        when simpleStageNeeded(MY_VESSEL) then {
          set CURR_STAGE to CURR_STAGE + 1.
          print "# BOOSTER SEPARATION".
          doSafeStage().

          // pickle the atmo stage
          when simpleStageNeeded(MY_VESSEL) then {
            set CURR_STAGE to CURR_STAGE + 1.
            print "# COLD STAGING ATMO STAGE".
            doSafeStage().
            wait 3.
            set CURR_STAGE to CURR_STAGE + 1.
            print "# UPPER STAGE IGNITION".
            doSafeStage().
            doStageDelay(45).

            when (MY_VESSEL:ALTITUDE > TGT_APO OR ETA:APOAPSIS > ETA:PERIAPSIS) then {
              set PAST_APO to true.
              print "# APOAPSIS - INVERTING PITCH GUIDANCE".
            }

            // pickle the upper stage
            when simpleStageNeeded(MY_VESSEL) then {
              set CURR_STAGE to CURR_STAGE + 1.
              print "# COLD STAGING UPPER STAGE".
              lock steering to PROGRADE.
              RCS on.
              doSafeStage().
              wait 3.
              set CURR_STAGE to CURR_STAGE + 1.
              print "# INSERTION STAGE IGNITION".
              doSafeStage().
              wait 3.
              RCS off.
              lock steering to heading(ascentHeading(TGT_INCL), circPitch(MY_VESSEL,PAST_APO),TGT_ASC_ROLL).
              doStageDelay(30).

              // wait for circular condition and terminate
              when MY_VESSEL:PERIAPSIS > TGT_PERI then {
                lock throttle to 0.
                print "# CIRCULARISED - INSERTION STAGE SHUTDOWN AT T+" + ROUND((TIME:SECONDS - START_TIME),0).
                print "# APOAPSIS " + ROUND(MY_VESSEL:APOAPSIS / 1000,1) + " KM".
                print "# PERIAPSIS " + ROUND(MY_VESSEL:PERIAPSIS / 1000,1) + " KM".
                print "# INCLINATION " + ROUND(MY_VESSEL:ORBIT:INCLINATION + 90,1) + " DEG".
                // final data point
                if (LOGGING_ENABLED) {
                  doTelemetry().
                }
                set AUTOPILOT to false.
              }
            }
          }
        }
      }
    }
  }

  // ### REF CODE ###
  if (false) {
    local validInput is false.
    print "# ADD MNV to MATCH APO AND [G] TO GO".
    until (validInput and HASNODE)  {
      set ch to terminal:input:getchar().
      if (ch = "G") set validInput to true.
    }
    executeManeuver(NEXTNODE, MY_VESSEL).
    set STAGE_ARM to false.
    set AUTOPILOT to false.
  }

}

// These can happen at any time during the flight
function doPreservedTriggers {

  // debug - prints maneuver time on AG9 toggle
  when AG9 then {
    if hasnode {
      local n is nextnode.
      //print "BurnTime:" + ROUND(maneuverBurnTime(n, MY_VESSEL),1) at (TERMINAL:WIDTH - 19,TERMINAL:HEIGHT - 7).
    //  print "BurnStart:" + ROUND(calculateStartTime(n, MY_VESSEL),1) at (TERMINAL:WIDTH - 20,TERMINAL:HEIGHT - 8).
    }
    AG9 off.
    return true.
  }

  // deploy accel-safe modules outside the atmo
  when MY_VESSEL:ALTITUDE > ATMO_BOUNDARY THEN {
    // deployAccelSafe().
  }

  // alert the highest Q
  when TOP_Q > MY_Q THEN {
    print "# THROUGH MAX Q " + ROUND(TOP_Q,1) + " KPA AT " + ROUND(MY_VESSEL:ALTITUDE / 1000,1) + " KM".
  }

}

function doTelemetry {
  if (CURR_STAGE >= 4) {
    set LOGGED_PITCH to circPitch(MY_VESSEL,PAST_APO).
  } else {
    set LOGGED_PITCH to ascentPitch(MY_VESSEL).
  }
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
    + MY_VESSEL:PERIAPSIS
    + ","
    + TGT_APO
    + ","
    + TGT_PERI
  to "TestFlight.csv".
}

// run last
function doFinalise {
  lock THROTTLE to 0.
  RCS on.
  lock steering to PROGRADE.
    wait 3.
  if (MY_VESSEL:PERIAPSIS > ATMO_BOUNDARY AND MY_VESSEL:APOAPSIS > ATMO_BOUNDARY AND DEPLOY_ORBIT_SAFE) {
    deployOrbitSafe().
  }
  // TODO clear flightplan
  set MY_VESSEL:control:neutralize to true.
  print "### PROGRAM COMPLETE ###".
}
