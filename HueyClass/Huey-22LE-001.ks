@LAZYGLOBAL OFF.

// include utilities once and only once
runoncepath("Utils").

print " ".
print "###################################".
print "# HUEY-22-LE-001 I FLIGHT PROGRAM #".
print "# 22T TEST MASS LEO INSERTION     #".
print "###################################".
print " ".

// ########## CONFIGURE LAUNCH #############
local TGT_APO is 180000. // target orbit to be circularised
local TGT_ROLL is 0. // TODO do a special roll program stage
local TGT_INCL is 118. // target orbital inclination, Canaveral lowest energy launch = 118
local TCOUNT is 5. // T-Minus countdown
local IGNITION_TIME IS 3. // Pre-Ignites RealFuels Engines if > 0
local MAX_Q is 50. // ideal max Q
local EST_CIRC_DV is 4000. // estimated circularisation dV.
// Stage List Parameters
// a: KSP Stage No
// b: Stage Type:
//    "PRE" - Pre-ignition
//    "TWR" - Tower Release
//    "STS" - Stage Separate
//    "STI" - Stage Ignite
//    "FAI" - Fairing Jettison (above atmo)
//    "MAN" - Manual Stage - pause auto staging
// c: Flight Phase:
//    "TO" - Tower
//    "AT" - Atmo Penetration
//    "AS" - Ascent
//    "BA" - Ballistic
//    "MN" - Maneuver
//    "OR" - Orbital
// d: Min Throttle for Stage
// e: Max Throttle for Stage
set StageList to list (
  list(8,"PRE","TO",0.2,0.2),
  list(7,"TWR","AT",0.2,0.5),
  list(6,"SEP","AS",0.2,1.0),
  list(5,"SEP","AS",0.2,1.0),
  list(4,"IGN","AS",0.2,1.0),
  list(3,"SEP","BA",0.2,1.0),
  list(2,"IGN","MN",0.2,1.0),
  list(1,"FAI","OR",0.2,1.0),
  list(0,"MAN","OR",0.2,1.0)
).
// ########## END CONFIGURE LAUNCH #############

// aliases
local Tp is 0.35. //0.01 Throttle PID
local Ti is 0.15. // 0.006 Throttle PID
local Td is 0.15. // 0.006 Throttle PID
local MY_VESSEL is SHIP. // safes against vehicle switch
local AUTOPILOT is true. // program will run until this is switched off
local STAGE_DISABLED is false. // is staging disabled
local LAUNCH_ALT is ROUND(MY_VESSEL:ALTITUDE,1). // alt of the control module
lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa. // dynamic pressure in kPa
local TPID is PIDLOOP(Tp, Ti, Td).
local CURR_STAGE to 0.

// RUN
doSetup().
doMain().
doFinalise().

// runs once
function doSetup {
  // global PID_THROTT is 0. // the PID controller's throttle setting
  set TPID:SETPOINT TO TGT_Q.
  set TPID:MAXOUTPUT to 1.
  set TPID:MINOUTPUT to -1.
  // surface key flight data
  print "Target Orbit: " + TGT_APO + "m".
  print "Target Orbital Inclination: " + TGT_INCL + " degrees".
  print "Launch Altitude: " + LAUNCH_ALT + "m".
}

// loops while program executing
function doMain {

  doPreservedTriggers().
  doFlightTriggers().

  until not AUTOPILOT {
    print "Q:" + ROUND(MY_Q,3) at (TERMINAL:WIDTH - 12,TERMINAL:HEIGHT - 2).
  }

}

// this is the main sequence of the flight plan
// driven by the StageList
function doFlightTriggers {

  // check for a tower phase
  when StageList[CURR_STAGE][2] is "TO" {
    print "# TOWER PHASE #".
    when StageList[CURR_STAGE][1] is "PRE" {

    }
  }


  // go ballistic once apo is at target orbit
  when MY_VESSEL:APOAPSIS >= TGT_APO then {
    print "# BALLISTIC PHASE #".
    lock THROTTLE to 0.
    // wait until above atmo to circularise
    when MY_VESSEL:ALTITUDE >= 140000 then {
      print "# CIRCULARISING #".
      doCircularization(MY_VESSEL, EST_CIRC_DV).
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

  // steering profiles
  when MY_VESSEL:ALTITUDE < 2 * LAUNCH_ALT THEN {
    // Tower Phase
    print "# TOWER PHASE #".
    stage.
    lock THROTTLE to ascentThrottle().
    lock steering to heading(TGT_INCL, 89.9).
    when MY_VESSEL:ALTITUDE > 2 * LAUNCH_ALT THEN {
      // cleared tower
      print "# ATMO PENETRATION PHASE #".
      lock steering to heading(ascentHeading(TGT_INCL), ascentPitch(MY_VESSEL),TGT_ROLL).
    }
  }

  // stage if needed throughout the program
  when stageNeeded(MY_VESSEL,StageList,CURR_STAGE) then {
    doSafeStage().
    set CURR_STAGE to CURR_STAGE - 1.

  }

}

// These can happen at any time during the flight
function doPreservedTriggers {

  // debug - prints maneuver time on AG9 toggle
  when AG9 then {
    if hasnode {
      local n is nextnode.
      print "BurnTime:" + ROUND(maneuverBurnTime(n, MY_VESSEL),1) at (TERMINAL:WIDTH - 19,TERMINAL:HEIGHT - 6).
      print "BurnStart:" + ROUND(calculateStartTime(n, MY_VESSEL),1) at (TERMINAL:WIDTH - 20,TERMINAL:HEIGHT - 7).
    }
    AG9 off.
    return true.
  }

  // deploy accel-safe modules outside the atmo
  when MY_VESSEL:ALTITUDE > 140000 THEN {
    // deployAccelSafe().
  }

}

// run last
function doFinalise {
  lock THROTTLE to 0.
  // TODO deployOrbitSafe().
  // TODO clear flightplan
  set MY_VESSEL:control:neutralize to true.
  print "### PROGRAM COMPLETE ###".
}
