// Standard AGs:
// AG1: Engine mode (F9 9/3/1)
// AG6: PV Panels
// AG7: Grid Fins toggle
// AG8: Vacuum Accel-safe Modules (e.g. fairings) - safe to do a burn after deployed
// AG9: Vacuum Accel-risk Modules (e.g. big antennas) - only deployed when there are no more burns
// AG0: KAL9000 power toggle

@LAZYGLOBAL OFF.

copypath("0:/common/LandingUtils", "").
runoncepath("LandingUtils").

print " ".
print "##########################################################".
print "# MISSION: " + MISSION_ID + "                                 #".
print "# SH PRECISION HOVERSLAM TESTFLIGHT 2                    #".
print "#                                                        #".
print "# Mission Objective:                                     #".
print "# - launch on norminal orbital ascent with 270T NRAP     #".
print "# - separate NRAP at nominal alt                         #".
print "# - perform boostback to precision location              #".
print "# - perform descent guidance to precision location       #".
print "# - perform hoverslam at precision location              #".
print "##########################################################".
print " ".
// CONFIGURE FLIGHT
local ATMO_BOUNDARY is 70000. // where does the atmo end
local SEP_APO is 70000. // apo at which stage separation occurs - TODO tie to booster fuel vs alt/distance
local SEP_DELAY is 10. // seconds to wait after staging before boostback
local TGT_APO is 100000. // target orbit to be circularised
local TGT_PERI is 90000. // target orbit to be circularised
local TGT_ASC_ROLL is 270. // do a special ascent roll program stage
local TGT_INCL is 90. // guidance heading (not inclination for now)
local LIMIT_Q is false. // limit Q to terminal velocity? CURRENTLY NONFUNCTIONAL
local MAX_Q is 40. // If limiting Q, limit to what?
local FULL_THROTT_OVER is 22000. // hacky way of preventing MECO before boosters
local EST_CIRC_DV is 2000. // estimated circularisation dV.
local DO_WARP is true. // set true to physics warp through boring bits
local WARP_SPEED is 2. // 1/2/3 corresponding to a 2x / 3x / 4x physics warp
local TCOUNT is 3. // T-Minus countdown
local TGANTRY is -1. // Gantry at T-Minus...
local TIGNITE is 1. // Ignite at T-Minus...
local Tmin is 0.1. // minimum throttle setting
local BOOST_APO is 12000. // after hover, how high should we boost
local BOOST_MAX_PITCH is 1. // Max pitchover (90 is vertical) during boostback
local DESCENT_MAX_PITCH is 70. // Max pitchover (90 is vertical) during aero descent
local HAGL is 250. // TARGET HOVER ALT METERS AGL
local GAGL is 500. // engage gear below on descent
local HOW_SUICIDAL is 0.98. // how late do you want to leave the burn? Close to but < 1.0 for max efficiency
local ENGINE_MODE_FACTOR is 4. // by what factor does thrust reduce changing mode? F9 = 3, SH = 4
local LZ to KSCLZ2. // where will we land?
local LOGGING_ENABLED is true. // log to CSV
// END CONFIGURE FLIGHT

// CONSTANTS, TUNING AND GLOBALS
local BOOSTBACK_READY is false.
local DESCENT_READY is false.
local MY_VESSEL is SHIP. // safes against vehicle switch
lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa. // dynamic pressure in kPa
lock TOP_Q to topQ(MY_Q).
local START_TIME to TIME:SECONDS.
local LOGGED_PITCH is 0.
local NEXT_LOG_TIME is TIME:SECONDS + 1.
local AGL_TWEAK is 1. // hoverslam AGL tweak - a little extra height to account for struts etc
local HTp is 0.05. // Hover Throttle P
local HTi is 0.1. // Hover Throttle I
local HTd is 0.15. // Hover Throttle D
local HTPID is PIDLOOP(HTp,HTi,HTd,-.1,.05). // adjust last two values for throttle speed
local AUTOPILOT is true. // program will run until this is switched off
local LAUNCH_AMSL is ROUND(ship:ALTITUDE,3). // AMSL of the control module
local LAUNCH_AGL is ROUND(MAX(0.001,(ALTITUDE-GEOPOSITION:TERRAINHEIGHT)),3). // AGL of the control module
local PREDICTED is 0. // predicted radalt with current accel + vel
local vAngle is 0. // angle from ship up to surface up
local Fg is 0. // force of gravity on the ship
local AGL is 0. // current AGL of the nozzles
local SLAM_THROTT is 0. // required throttle to safely slam
if archive:exists("TestFlight.csv") {
  archive:delete("TestFlight.csv").
}
local LOGFILE to archive:create("TestFlight.csv").

// RUN
doSetup().
doMain().
doFinalise().
// -------------------------------
// FUNCTIONS ONLY BELOW THIS POINT

// the main sequence of the flight plan
function doFlightTriggers {

  when true THEN {
    doTowerPhase().

    // clear of the tower, ascend
    when AGL > (2 * LAUNCH_AGL) THEN {
      logMessage(LOGADVISORY,"TOWER CLEAR").
      doAscentPhase().

      // ascent complete, go ballistic and separate
      when (MY_VESSEL:APOAPSIS > SEP_APO) then {
        doBallisticPhase().

        // separated and clear, do boostback
        when (BOOSTBACK_READY) then {
          doBoostbackPhase().

          // bostback complete, atmo descent
          when (DESCENT_READY) then {
            doDescentPhase().

            // time to slam
            when (SLAM_THROTT > HOW_SUICIDAL) then {
              doHoverslam().

              // gear when approaching ground
              when distanceToGround(LAUNCH_AGL,AGL_TWEAK) < GAGL then {
                doLandingGear().

                when (ship:status = "LANDED" or AGL < 1) then {
                 doTouchDown().
                }
              }
            }
          }
        }
      }
    }
  }
}

function doTowerPhase {
  logMessage(LOGMAJOR,"TOWER PHASE").
  logMessage(LOGADVISORY, "Target Orbit: A: " + TGT_APO/1000 + "km / P: " + TGT_PERI/1000 + "km").
  logMessage(LOGADVISORY, "Target Orbital Inclination: " + TGT_INCL + " deg").
  logMessage(LOGADVISORY, "Limiting to Terminal Velocity: " + LIMIT_Q).
  switch to archive.
  if (LOGGING_ENABLED) {
    log logpid() + ",,,,," to "TestFlight.csv".
    log "MET,ALT,PITCH,Q,APO,PERI,TGTAPO,TGTPERI" to "TestFlight.csv".
  }
  doCountdown(TCOUNT, TIGNITE, Tmin, TGANTRY).
  lock throttle to towerThrottle().
  logMessage(LOGADVISORY,"LIFTOFF").
  lock steering to up.
  wait 1.
  rcs off.
}

function doEngineMode {
  toggle AG1.
  logMessage(LOGADVISORY,"HTECO").
}

function doAscentPhase {
  logMessage(LOGMAJOR,"ASCENT PHASE").
  if DO_WARP { set warp to WARP_SPEED. }
  lock steering to heading(ascentHeading(TGT_INCL), ascentPitch(MY_VESSEL),TGT_ASC_ROLL).
  lock throttle to 1.
  rcs off.
  // alert the highest Q
  when TOP_Q > (MY_Q + 1) THEN {
    logMessage(LOGADVISORY,"THROUGH MAX Q " + ROUND(TOP_Q,1) + " KPA AT " + ROUND(MY_VESSEL:ALTITUDE / 1000,1) + " KM").
  }
  logMessage(LOGADVISORY,"SEPARATING AT " + SEP_APO + "M APO").
}

function doBallisticPhase {
  logMessage(LOGMAJOR,"BALLISTIC PHASE").
  if DO_WARP { set warp to 0. }
  lock throttle to 0.
  logMessage(LOGADVISORY,"MECO").
  doSafeStage().
  logMessage(LOGADVISORY,"UPPER STAGE SEPARATION").
  rcs on.
  lock steering to SRFRETROGRADE.
  wait SEP_DELAY.
  set BOOSTBACK_READY to true.
}

function doBoostbackPhase {
  logMessage(LOGMAJOR,"BOOSTBACK PHASE").
  if DO_WARP { set warp to 0. }
  lock steering to boostbackSteering(LZ,BOOST_MAX_PITCH).
  lock throttle to 0.1.
  wait 10.
  lock throttle to 1.
  when boostbackClose(LZ) then {
    lock throttle to 0.1.
    when boostbackComplete(LZ) then {
      lock throttle to 0.
      logMessage(LOGADVISORY,"BOOSTBACK COMPLETE").
      set DESCENT_READY to true.
    }
  }
}

function doDescentPhase {
  logMessage(LOGMAJOR,"DESCENT PHASE").
  if DO_WARP { set warp to 0. }
  // lock steering to atmosphericDescentSteering(LZ).
  lock steering to landingSteering(LZ,AGL).
  rcs on.
  doGridfins(true).
  doEngineMode().
}

function doHoverslam {
  if DO_WARP { set warp to 0. }
  logMessage(LOGMAJOR,"HOVERSLAM PHASE").
  logMessage(LOGADVISORY,"ME IGNITION").
  lock steering to landingSteering(LZ,AGL).
  lock throttle to SLAM_THROTT.
}

function doLandingGear {
  gear on.
  logMessage(LOGADVISORY,"LANDING STRUTS").
}

function doTouchDown {
  logMessage(LOGADVISORY,"TOUCHDOWN").
  lock throttle to 0.
  doGridfins(false).
  lock steering to up.
  set AUTOPILOT to false.
  logMessage(LOGADVISORY,"GUIDANCE OFFLINE - OK").
  wait 3.
  rcs off.
}

// used for triggers that can run multiple times
// note that preserve has been replaced by return - https://ksp-kos.github.io/KOS/language/flow.html
function doPreservedTriggers {

}

function doTelemetry {
  // logConsole parameters mType,msg,val,index.
  if addons:tr:hasImpact {
    logConsole(LOGTELEMETRY,"TGTLAT",ROUND(addons:tr:impactpos:LAT,6),10).
    logConsole(LOGTELEMETRY,"TGTLNG",ROUND(addons:tr:impactpos:LNG,6),9).
  }
  logConsole(LOGTELEMETRY,"Q",ROUND(MY_Q,1),8).
  logConsole(LOGTELEMETRY,"Mass",ROUND(ship:mass,1),7).
  logConsole(LOGTELEMETRY,"Vv",ROUND(ship:verticalspeed,3),6).
  logConsole(LOGTELEMETRY,"vAngle",ROUND(vAngle,3),5).
  logConsole(LOGTELEMETRY,"Fg",ROUND(Fg,3),4).
  logConsole(LOGTELEMETRY,"AGL",ROUND(AGL, 1),3).
  logConsole(LOGTELEMETRY,"AMSL",ROUND(ship:ALTITUDE - LAUNCH_AGL,3),2).
  logConsole(LOGTELEMETRY,"SLAMT",ROUND(SLAM_THROTT, 3),1).
}

// runs once at the start of the script
function doSetup {
  neutraliseRoll().
  set HTPID:SETPOINT TO HAGL.
  lock vAngle to VANG(ship:facing:forevector, ship:up:forevector).
  lock Fg to (body:mu / body:radius^2) * mass.
  lock AGL to baseRadalt(LAUNCH_AGL).
  lock SLAM_THROTT to min (1, stoppingDistance() / distanceToGround(LAUNCH_AGL,AGL_TWEAK)).
  set kuniverse:TimeWarp:MODE to "PHYSICS".
  // surface key flight data that is mission-agnostic
  logMessage(LOGADVISORY,"Launch AMSL: " + LAUNCH_AMSL + "m").
  logMessage(LOGADVISORY,"Launch AGL: " + LAUNCH_AGL + "m").
  logMessage(LOGADVISORY,"Wet Mass: " + ROUND(ship:wetmass,1) + "T").
  logMessage(LOGADVISORY,"Dry Mass: " + ROUND(ship:drymass,1) + "T").
  if ADDONS:TR:AVAILABLE {
      logMessage(LOGADVISORY,"Trajectories available - OK").
  } else {
      logMessage(LOGERROR,"Trajectories is not available.").
  }
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
  // TODO clear flightplan
  set ship:control:neutralize to true.
  logMessage(LOGMAJOR,"PROGRAM COMPLETE").
  until false {
    wait 1.
  }
}

function logTelemetry {
    set LOGGED_PITCH to ascentPitch(MY_VESSEL).
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
