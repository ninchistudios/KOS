@LAZYGLOBAL OFF.

// include utilities once and only once
runoncepath ("0:/Utils.ks").

print " ".
print "#############################################".
print "# POLAR CONTRACT 02 (PROBO) ORBITAL PROGRAM #".
print "#############################################".
print " ".

// CONFIGURE LAUNCH
local TGT_APO is 90000. // target orbit to be circularised
local TGT_INCL is 0. // target orbital inclination
local TCOUNT is 3. // T-Minus countdown
local Tp is 0.35. //0.01
local Ti is 0.15. // 0.006
local Td is 0.15. // 0.006
local TGT_Q is 99. // ideal max Q, 26=375dV
local EST_CIRC_DV is 900. // estimated circularisation dV.

// aliases
local MY_VESSEL is SHIP. // safes against vehicle switch
local AUTOPILOT is true. // program will run until this is switched off
local STAGE_ARM is true. // is staging armed
local LAUNCH_ALT is ROUND(MY_VESSEL:ALTITUDE,1). // alt of the control module
lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa. // dynamic pressure in kPa
local TPID is PIDLOOP(Tp, Ti, Td).

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
  doCountdown(TCOUNT).
}

// loops while program executing
function doMain {

  doPreservedTriggers().
  doFlightTriggers().

  until not AUTOPILOT {
    print "Q:" + ROUND(MY_Q,3) at (TERMINAL:WIDTH - 12,TERMINAL:HEIGHT - 2).
  }

}

// set up ascent triggers
// this is the main sequence of the flight plan
function doFlightTriggers {
  // go ballistic once apo is at target orbit
  when MY_VESSEL:APOAPSIS >= TGT_APO then {
    print "# BALLISTIC PHASE #".
    lock THROTTLE to 0.
    // wait until above atmo to circularise
    when MY_VESSEL:ALTITUDE >= 70000 then {
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
  // blow accel-safe modules outside the atmo
  when MY_VESSEL:ALTITUDE > 60000 THEN {
    deployAccelSafe().
  }
  // steering profiles
  when MY_VESSEL:ALTITUDE < 2 * LAUNCH_ALT THEN {
    // Tower Phase
    print "# TOWER PHASE #".
    lock THROTTLE to ascentThrottle().
    lock steering to heading(TGT_INCL, 89.9).
    when MY_VESSEL:ALTITUDE > 2 * LAUNCH_ALT THEN {
      // cleared tower
      print "# ATMO PENETRATION PHASE #".
      lock steering to heading(ascentHeading(TGT_INCL), ascentPitch(MY_VESSEL)).
    }
  }
}

function doPreservedTriggers {
  // stage if needed throughout the program
  when stageNeeded(MY_VESSEL) then {
    doSafeStage().
    return STAGE_ARM.
  }
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
}

// run last
function doFinalise {
  lock THROTTLE to 0.
  deployOrbitSafe().
  // TODO clear flightplan
  set MY_VESSEL:control:neutralize to true.
  print "### PROGRAM COMPLETE ###".
}
