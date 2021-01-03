// Orbital Program for Polar Contract 01 (STEADLER)
// switch to 0.

print " ".
print "################################################".
print "# POLAR CONTRACT 01 (STEADLER) ORBITAL PROGRAM #".
print "################################################".
print " ".

// CONFIGURE ORBIT
set TGT_APO to 82000.
print "Target Apoapsis: " + TGT_APO + "m".
set TGT_PERI to 81000.
print "Target Periapsis: " + TGT_PERI + "m".
set TGT_INCL to 351.
print "Target Inclination: " + TGT_INCL + " degrees".
set TCOUNT to 3.
// CONFIGURE THROTTLE PID
SET Kp TO 0.35. //0.01
SET Ki TO 0.15. // 0.006
SET Kd TO 0.15. // 0.006
set TGT_Q to 26.
// CONFIGURE DEPLOYABLE FAIRINGS
// For now, put all fairings in action group 1
// set FAIRS to list("Fairing1", "Fairing2", "Fairing3").

// Main Loop
doSetup().
until COMPLETE {
  doMain().
}

// run once
function doSetup {
  set COMPLETE to false.
  set ASCENT_PHASE to false.
  set BALLISTIC_PHASE to false.
  set CIRCLE_PHASE to false.
  set FAIRINGS_DEPLOYED to false.
  set MY_VESSEL to SHIP.
  set PREV_THRUST to 0.
  set PID_THROTT to 1.
  lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa.
  SET PID TO PIDLOOP(Kp, Kp, Kd).
  // SET PID:SETPOINT TO 1.2.
  SET PID:SETPOINT TO TGT_Q.
  set PID:MAXOUTPUT to 1.
  set PID:MINOUTPUT to -1.
  // SET G TO KERBIN:MU / KERBIN:RADIUS^2.
  // LOCK ACCVEC TO MY_VESSEL:SENSORS:ACC - MY_VESSEL:SENSORS:GRAV.
  // LOCK GFORCE TO ACCVEC:MAG / G.
  lock MY_ALT to MY_VESSEL:ALTITUDE.
  set LAUNCH_ALT to ROUND(MY_ALT,1).
  print "Launch Alt: " + LAUNCH_ALT + "m".
  lock throttle to 1. // need to manage by alt & Q
  lock CLEAR_TOWER to (MY_ALT > (LAUNCH_ALT * 2)).
  doCountdown().
}

// run until COMPLETE
function doMain {

  // TOWER PHASE
  until ASCENT_PHASE {

    UNTIL MY_VESSEL:MAXTHRUST > 0 {
      lock targetPitch to 88.963.
      lock steering to heading(TGT_INCL, targetPitch).
      WAIT 0.5.
      safeStage().
    }

    if (CLEAR_TOWER) {
      print "# CLEAR TOWER - ASCENT PHASE #".
      set ASCENT_PHASE to true.
    }
  }

  // ASCENT PHASE
  until BALLISTIC_PHASE {

    checkFlameOut().

    // throttle & pitch control
    if (MY_ALT < 1000) {
      lock targetPitch to 88.963.
      lock throttle to 1.
    } else if (Apoapsis < TGT_APO) {
      lock targetPitch to 88.963 - (1.03287 * MY_ALT^0.39). // MY_ALT^0.409511
      doThrottlePID().
    } else {
      // lock targetPitch to MY_VESSEL:PROGRADE.
      lock throttle to 0.
      print "# APOAPSIS AT TARGET - BALLISTIC PHASE #".
      set BALLISTIC_PHASE to true.
    }

  }

  until CIRCLE_PHASE {

    until FAIRINGS_DEPLOYED {
      if (MY_VESSEL:ALTITUDE > 60000) deployFairings().
    }

    if (MY_VESSEL:ALTITUDE > 70000) {
      set CIRCLE_PHASE to true.
    }

  }

  executeCircularize().

  // program complete
  set THROTTLE to 0.
  set COMPLETE to true.
  SAS on.
  set MY_VESSEL:CONTROL:NEUTRALIZE TO TRUE.
  print "### PROGRAM COMPLETE ###".
}

function checkFlameOut {
  // stage if a stage flames out
  set CURRENT_THRUST to MY_VESSEL:AVAILABLETHRUST.
  if CURRENT_THRUST < (PREV_THRUST - 10) {
    safeStage().
  }
  set PREV_THRUST to CURRENT_THRUST.
}

function executeCircularize {
  local mnv is createManeuver(time:seconds + eta:APOAPSIS, 0, 0, 1000).
  tunePrograde(mnv).
  lock steering to mnv:burnvector.
  wait until eta:APOAPSIS < getBurnTime().
  lock throttle to 1.
  until (PERIAPSIS > TGT_PERI) {
    checkFlameOut().
    adjustCircThrottle().
  }
  set throttle to 0.
  set steering to MY_VESSEL:prograde.
  removeManeuver(mnv).
  SAS on.
  print "# CIRCULARIZE COMPLETE #".
}

function adjustCircThrottle {
  if (MY_VESSEL:PERIAPSIS < (0.6 * TGT_PERI)) {
    set throttle to 1.
  } else {
    set throttle to MAX(0.1, MAX(1, TGT_PERI / MY_VESSEL:PERIAPSIS)).
  }
}

function tunePrograde {
  parameter mnv.
  local tuneFactor is 100.
  local tooLow is true.
  until ((mnv:orbit:PERIAPSIS = TGT_PERI) or tuneFactor < 1) {
    print "DEBUG: mnv peri:" + mnv:orbit:PERIAPSIS.
    if mnv:orbit:PERIAPSIS > TGT_PERI {
      if tooLow set tuneFactor to tuneFactor / 2.
      set tooLow to false.
      set mnv:prograde to mnv:prograde - tuneFactor.
    }
    if mnv:orbit:PERIAPSIS < TGT_PERI {
      if (tooLow = false) set tuneFactor to tuneFactor / 2.
      set tooLow to true.
      set mnv:prograde to mnv:prograde + tuneFactor.
    }
  }
  print "DEBUG: mnv peri:" + mnv:orbit:PERIAPSIS.
}

function createManeuver {
  parameter utime, rad, norm, pro.
  local mnv1 is node(utime, rad, norm, pro).
  addManeuver(mnv1).
  return mnv1.
}

function addManeuver {
  parameter mnv.
  add mnv.
}

function removeManeuver {
  parameter mnv.
  remove mnv.
}

function getBurnTime {
  set orbitalVelocity to ship:body:radius * sqrt(9.8/(ship:body:radius + apoapsis)).
  set deltaA to maxthrust/mass.
  set apVelocity to sqrt(ship:body:mu * ((2/(ship:body:radius + apoapsis))-(1/ship:obt:semimajoraxis))).
  set deltaV to (orbitalVelocity - apVelocity).
  set timeToBurn to deltaV/deltaA.
  return timeToBurn.
}

function deployFairings {
  AG1 ON.
  print "# FAIRINGS DEPLOYED #".
  set FAIRINGS_DEPLOYED to true.
}

function safeStage {
  wait until STAGE:READY.
  STAGE.
  set PREV_THRUST to MY_VESSEL:AVAILABLETHRUST.
}

function doCountdown {

  PRINT "# T MINUS".
  FROM {local countdown is TCOUNT.} UNTIL countdown = 0 STEP {SET countdown to countdown - 1.} DO {
    PRINT "# ... " + countdown.
    WAIT 1. // pauses the script here for 1 second.
  }
  print "# IGNITION - TOWER PHASE #".
}

function doThrottlePID {
  // LOCK THROTTLE TO PID_THROTT.
  SET PID_THROTT TO PID_THROTT + PID:UPDATE(TIME:SECONDS, MY_Q).
  if (PID_THROTT < 0) {
    set PID_THROTT to 0.
  } else if (PID_THROTT > 1) {
    set PID_THROTT to 1.
  }
  set THROTTLE to PID_THROTT.
  // pid:update() is given the input time and input and returns the output. gforce is the input.
  WAIT 0.001.
  // print "DEBUG: Q=" + MY_Q + " OUTPUT=" + ROUND(PID:OUTPUT,2) + " PID Throttle:" + PID_THROTT.
}


// this is a basic ascent curve, check ok
//

// stage.
// wait 2.
// stage.
// UNTIL SHIP:MAXTHRUST = 0 {
    // WAIT 1.
    // PRINT "NOMINAL".
// }
