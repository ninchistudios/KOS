// Standard AGs:
// AG1: Engine mode (F9 9/3/1)
// AG6: PV Panels
// AG7: Grid Fins toggle
// AG8: Vacuum Accel-safe Modules (e.g. fairings) - safe to do a burn after deployed
// AG9: Vacuum Accel-risk Modules (e.g. big antennas) - only deployed when there are no more burns

@LAZYGLOBAL OFF.

copypath("0:/common/LandingUtils", "").
runoncepath("LandingUtils").

print " ".
print "##########################################################".
print "# MISSION: 2021-SH-002-T                                 #".
print "# SH PRECISION HOVERSLAM TESTFLIGHT 1                    #".
print "#                                                        #".
print "# Mission Objective:                                     #".
print "# - launch to altitude                                   #".
print "# - perform hoverslam at precision location              #".
print "##########################################################".
print " ".
// CONFIGURE FLIGHT
local DO_WARP is false. // set true to physics warp through boring bits
local WARP_SPEED is 2. // 1/2/3 corresponding to a 2x / 3x / 4x physics warp
local TCOUNT is 3. // T-Minus countdown
local TGANTRY is -1. // Gantry at T-Minus...
local TIGNITE is 1. // Ignite at T-Minus...
local Tmin is 0.1. // minimum throttle setting
local BOOST_APO is 12000. // after hover, how high should we boost
local BOOST_MAX_PITCH is 80. // Max pitchover (90 is vertical) during boostback
local DESCENT_MAX_PITCH is 70. // Max pitchover (90 is vertical) during aero descent
local HAGL is 250. // TARGET HOVER ALT METERS AGL
local GAGL is 500. // engage gear below on descent
local HOW_SUICIDAL is 0.9. // how late do you want to leave the burn? Close to but < 1.0 for max efficiency
local ENGINE_MODE_FACTOR is 4. // by what factor does thrust reduce changing mode? F9 = 3, SH = 4
local LZ to KSCLZ1. // where will we land?
// END CONFIGURE FLIGHT

// CONSTANTS, TUNING AND GLOBALS
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

    // clear of the tower
    when AGL > (2 * LAUNCH_AGL) THEN {
      logMessage(LOGADVISORY,"TOWER CLEAR").
      doBoostPhase().

      // reduce thrust to 3 engines at 1/3 throttle
      when hoverThrottle(AGL,HAGL,Tmin) < (1/ENGINE_MODE_FACTOR) then {
        doEngineMode().
      }

      // boost complete, go ballistic
      when (ship:APOAPSIS > BOOST_APO) then {
        doBallisticPhase().

        // descending at top of boost
        when ship:verticalSpeed < 0 then {
          doDescentPhase().

          // throttle up
          when (SLAM_THROTT > HOW_SUICIDAL) then {
            doHoverslam().

            // gear when approaching ground
            when distanceToGround(LAUNCH_AGL,AGL_TWEAK) < GAGL then {
              doLandingGear().

              when (ship:status = "LANDED") then {
               doTouchDown().
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
  logMessage(LOGADVISORY,"Target Boost Apo: " + BOOST_APO + "m").
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

function doBoostPhase {
  logMessage(LOGMAJOR,"BOOST PHASE").
  // if DO_WARP { set warp to 0. }
  lock steering to heading(315, 89).
  lock throttle to 1.
  rcs off.
}

function doBallisticPhase {
  logMessage(LOGMAJOR,"BALLISTIC PHASE").
  lock steering to up.
  lock throttle to 0.
  logMessage(LOGADVISORY,"MECO").
  rcs on.
}

function doDescentPhase {
  logMessage(LOGMAJOR,"DESCENT PHASE").
  lock steering to atmosphericDescentSteering(LZ).
  rcs off.
  AG7 on.
  logMessage(LOGADVISORY,"GRIDFINS").
}

function doHoverslam {
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
  AG7 off. // gridfins
  lock steering to up.
  set AUTOPILOT to false.
  logMessage(LOGADVISORY,"GUIDANCE OFFLINE - OK").
}

// used for triggers that can run multiple times
// note that preserve has been replaced by return - https://ksp-kos.github.io/KOS/language/flow.html
function doPreservedTriggers {

}

function doTelemetry {
  // logConsole parameters mType,msg,val,index.
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
  logMessage(LOGADVISORY,"Wet Mass: " + ROUND(ship:wetmass,1)).
  logMessage(LOGADVISORY,"Dry Mass: " + ROUND(ship:drymass,1)).
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
  }
}

// run last
function doFinalise {
  lock THROTTLE to 0.
  // TODO clear flightplan
  // set ship:control:neutralize to true.
  logMessage(LOGMAJOR,"PROGRAM COMPLETE").
  until false {
    wait 1.
  }
}
