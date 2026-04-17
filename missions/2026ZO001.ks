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
print "# MISSION: 2026ZO001                                     #".
print "# Contract: Zoe-class Orbital Development                #".
print "#                                                        #".
print "# Mission Objective:                                     #".
print "# - demonstrate smooth ascent to accurate apogee         #".
print "##########################################################".
print " ".

// CONFIGURE FLIGHT
// ------------------------------------------------------------
// VEHICLE AND PAD GEOMETRY
local CFG_AGL_BARE is 17.7.

// ASCENT TARGETS
local CFG_CUT_APO is 215000.
local CFG_TARGET_HEADING is 90.
local CFG_TARGET_PITCH is 89.9.
local CFG_PITCH_START_ALT is 1000.

// DESCENT PROFILE
local CFG_CHUTE_STAGE_ALT is 14000.
local CFG_DESCENT_PITCH is 90.

// COUNTDOWN AND ENGINE START SAFETY
local CFG_TCOUNT is 7.
local CFG_TIGNITE is 5.
local CFG_TTHROTTLE is 1.
local CFG_TGANTRY is 0.
local CFG_TMIN is 0.32.
local CFG_ENGINE_START_SPOOL_TIME is 2.45.
local CFG_ENGINE_START_TMIN_EXPECTED_FRAC is 0.95.
local CFG_ENGINE_START_FULL_EXPECTED_FRAC is 0.90.

// WARP, TELEMETRY, LOGGING
local CFG_DO_WARP is false.
local CFG_WARP_SPEED is 3.
local CFG_TELEMETRY_ENABLED is true.
local CFG_LOGGING_ENABLED is true.

// PAYLOAD DEPLOYMENT SAFETY
local CFG_DEPLOY_ACCEL_SAFE is true.
local CFG_DEPLOY_ACCEL_SAFE_ALT is 100000.
local CFG_DEPLOY_ACCEL_SAFE_Q is 0.

// MISSION SUCCESS CRITERIA
local CFG_ENGINE_NOMINAL_BURN is 83.0.
local CFG_MISSION_MIN_APO is 200000.
local CFG_MISSION_MIN_AVIONICS_TIME is 50.
local CFG_MISSION_MAX_APO_ERROR is 15000.
// END CONFIGURE FLIGHT

// CONSTANTS AND GLOBALS
local LAUNCH_AMSL is ROUND(SHIP:ALTITUDE,3).
local LAUNCH_AGL is ROUND(MAX(0.001,(ALTITUDE-GEOPOSITION:TERRAINHEIGHT)),3).
local LAUNCH_GEO is SHIP:GEOPOSITION.
local MY_VESSEL is SHIP.

lock MY_Q to MY_VESSEL:Q * constant:ATMtokPa.
lock TOP_Q to topQ(MY_Q).
lock DOWNRANGE to launchDownrangeMeters(MY_VESSEL, LAUNCH_GEO).
global START_TIME is TIME:SECONDS.

local LOGGED_PITCH is 0.
local NEXT_LOG_TIME is TIME:SECONDS + 1.
local AUTOPILOT is true.
local ACCEL_SAFE_DEPLOYED is false.
local AGL is 0.
local MAX_PERIAPSIS is 0.
local TARGET_APO_REACHED is false.

local GOAL_MIN_APO is false.
local GOAL_TARGET_APO is false.
local GOAL_SUBORBITAL is true.
local GOAL_AVIONICS_TIME is false.

local MYLOGFILE is "".

// RUN
doSetup().
doMain().
doFinalise().
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
			when (stageNeeded(MY_VESSEL) or MY_VESSEL:PERIAPSIS > 0 or MY_VESSEL:APOAPSIS > CFG_CUT_APO) then {
				doMECO().

				// ballistic coast - wait for impact
				when (MY_VESSEL:STATUS = "LANDED" or MY_VESSEL:STATUS = "SPLASHED") then {
					doTouchDown().
				}
			}
		}
	}
}

function doMissionGoals {
	when MY_VESSEL:ALTITUDE >= CFG_MISSION_MIN_APO THEN {
		set GOAL_MIN_APO to true.
	}

	when MY_VESSEL:APOAPSIS >= (CFG_MISSION_MIN_APO - CFG_MISSION_MAX_APO_ERROR) and MY_VESSEL:APOAPSIS <= (CFG_MISSION_MIN_APO + CFG_MISSION_MAX_APO_ERROR) THEN {
		set GOAL_TARGET_APO to true.
		set TARGET_APO_REACHED to true.
	}

	when MY_VESSEL:PERIAPSIS > MAX_PERIAPSIS THEN {
		set MAX_PERIAPSIS to MY_VESSEL:PERIAPSIS.
	}

	when MAX_PERIAPSIS > 140000 THEN {
		set GOAL_SUBORBITAL to false.
	}

	when currentMETseconds() > CFG_MISSION_MIN_AVIONICS_TIME THEN {
		set GOAL_AVIONICS_TIME to true.
	}
}

function doTowerPhase {
	logMessage(LOGMAJOR,"TOWER PHASE").
	logMessage(LOGADVISORY, "Target Heading: " + CFG_TARGET_HEADING + " / Ascent Pitch: " + CFG_TARGET_PITCH).
	logMessage(LOGADVISORY, "Pitch Program Alt: " + CFG_PITCH_START_ALT/1000 + "km").

	if (CFG_LOGGING_ENABLED and MYLOGFILE <> "") {
		log logpid() + ",,,," to MYLOGFILE.
		log "MET,ALT,PITCH,Q,APO,DOWNRANGE" to MYLOGFILE.
	}

	if not doCountdownWithEngineStartScrub(
		CFG_TCOUNT,
		CFG_TIGNITE,
		CFG_TMIN,
		CFG_TGANTRY,
		CFG_TTHROTTLE,
		MY_VESSEL,
		CFG_ENGINE_START_SPOOL_TIME,
		CFG_ENGINE_START_TMIN_EXPECTED_FRAC,
		CFG_ENGINE_START_FULL_EXPECTED_FRAC
	) {
		set AUTOPILOT to false.
		return.
	}

	lock THROTTLE to towerThrottle().
	logMessage(LOGADVISORY,"LIFTOFF").
	lock STEERING to up.
	WAIT 1.
	RCS OFF.
}

function doAscentPhase {
	logMessage(LOGMAJOR,"ASCENT PHASE").
	if CFG_DO_WARP { set warp to CFG_WARP_SPEED. }

	lock THROTTLE to 1.
	RCS OFF.

	when (MY_VESSEL:ALTITUDE >= CFG_DEPLOY_ACCEL_SAFE_ALT and MY_Q <= CFG_DEPLOY_ACCEL_SAFE_Q and CFG_DEPLOY_ACCEL_SAFE and not ACCEL_SAFE_DEPLOYED) then {
		deployAccelSafe().
		set ACCEL_SAFE_DEPLOYED to true.
	}

	when TOP_Q > (MY_Q + 1) THEN {
		logMessage(LOGADVISORY,"THROUGH MAX Q " + ROUND(TOP_Q,1) + " KPA AT " + ROUND(MY_VESSEL:ALTITUDE / 1000,1) + " KM").
	}

	when MY_VESSEL:ALTITUDE >= CFG_PITCH_START_ALT THEN {
		logMessage(LOGADVISORY,"Pitch Program - HDG " + CFG_TARGET_HEADING + " Pitch " + CFG_TARGET_PITCH).
		lock STEERING to heading(CFG_TARGET_HEADING, CFG_TARGET_PITCH).
	}

	when getResourceAmount("ELECTRICCHARGE") < 0.1 THEN {
		logMessage(LOGERROR,"LOW ELECTRICITY").
	}
}

function doMECO {
	logMessage(LOGMAJOR,"MECO").
	if CFG_DO_WARP { set warp to 0. }

	lock THROTTLE to 0.
	local met is ROUND(TIME:SECONDS - START_TIME, 1).
	logMessage(LOGADVISORY,"BURNOUT MET+" + met + "s / DOWNRANGE " + ROUND(DOWNRANGE/1000,1) + "km").

	if (met < CFG_ENGINE_NOMINAL_BURN or MY_VESSEL:APOAPSIS < CFG_MISSION_MIN_APO) {
		logMessage(LOGERROR,"ENGINE OUT OF NOMINAL").
	} else {
		logMessage(LOGADVISORY,"ENGINE BURN NOMINAL").
	}

	lock STEERING to up.
	logMessage(LOGADVISORY,"BALLISTIC").

	when (MY_VESSEL:ALTITUDE >= CFG_DEPLOY_ACCEL_SAFE_ALT and MY_Q <= CFG_DEPLOY_ACCEL_SAFE_Q and not ACCEL_SAFE_DEPLOYED) then {
		logMessage(LOGADVISORY,"STAGING AVIONICS AND PAYLOAD").
		doSafeStage().
		set ACCEL_SAFE_DEPLOYED to true.
	}

	when (MY_VESSEL:ALTITUDE <= CFG_CHUTE_STAGE_ALT and MY_VESSEL:VERTICALSPEED < 0) then {
		logMessage(LOGADVISORY,"CHUTE DEPLOYMENT AT " + ROUND(CFG_CHUTE_STAGE_ALT/1000,1) + "KM").
		doSafeStage().
	}
}

function doTouchDown {
	logMessage(LOGMAJOR,"SPLASHDOWN / TOUCHDOWN").
	logMessage(LOGADVISORY,"FINAL DOWNRANGE: " + ROUND(DOWNRANGE/1000,1) + "km").
	logMessage(LOGADVISORY,"MAX APO: " + ROUND(MY_VESSEL:APOAPSIS/1000,1) + "km").
	lock THROTTLE to 0.
	set MY_VESSEL:CONTROL:NEUTRALIZE to true.
	set AUTOPILOT to false.
	logMessage(LOGADVISORY,"GUIDANCE OFFLINE - OK").
}

function doTelemetry {
	if CFG_TELEMETRY_ENABLED {
		logConsole(LOGTELEMETRY,"Q",ROUND(MY_Q,1),6).
		logConsole(LOGTELEMETRY,"Mass",ROUND(MY_VESSEL:MASS,1),5).
		logConsole(LOGTELEMETRY,"Vv",ROUND(MY_VESSEL:VERTICALSPEED,1),4).
		logConsole(LOGTELEMETRY,"APO",ROUND(MY_VESSEL:APOAPSIS/1000,1),3).
		logConsole(LOGTELEMETRY,"ALT",ROUND(MY_VESSEL:ALTITUDE/1000,1),2).
		logConsole(LOGTELEMETRY,"DWNRNG",ROUND(DOWNRANGE/1000,1),1).
	}
}

function doSetup {
	neutraliseRoll().
	lock AGL to baseRadalt(CFG_AGL_BARE).
	set kuniverse:TimeWarp:MODE to "PHYSICS".

	if CFG_LOGGING_ENABLED {
		if archive:exists("TestFlight.csv") {
			archive:delete("TestFlight.csv").
		}
		set MYLOGFILE to archive:create("TestFlight.csv").
	} else {
		set MYLOGFILE to "".
	}

	logMessage(LOGADVISORY,"Launch AMSL: " + LAUNCH_AMSL + "m").
	logMessage(LOGADVISORY,"Launch AGL: " + LAUNCH_AGL + "m").
	logMessage(LOGADVISORY,"Hardware AGL: " + CFG_AGL_BARE + "m").
	logMessage(LOGADVISORY,"Wet Mass: " + ROUND(MY_VESSEL:WETMASS,1) + "T").
	logMessage(LOGADVISORY,"Dry Mass: " + ROUND(MY_VESSEL:DRYMASS,1) + "T").
}

function doMain {
	doFlightTriggers().
	until not AUTOPILOT {
		doTelemetry().
		if (CFG_LOGGING_ENABLED and MYLOGFILE <> "") {
			if TIME:SECONDS >= NEXT_LOG_TIME {
				set NEXT_LOG_TIME to NEXT_LOG_TIME + 1.
				logTelemetry().
			}
		}
		WAIT 0.
	}
}

function doFinalise {
	lock THROTTLE to 0.
	set MY_VESSEL:CONTROL:NEUTRALIZE to true.
	logMessage(LOGMAJOR, "FINAL MET: " + currentMETFormatDHMS()).
	logMessage(LOGMAJOR, "MISSION GOALS:").
	logMessage(LOGMAJOR, " - Min Apoapsis: " + GOAL_MIN_APO).
	logMessage(LOGMAJOR, " - Target Apoapsis Band: " + GOAL_TARGET_APO).
	logMessage(LOGMAJOR, " - Suborbital: " + GOAL_SUBORBITAL).
	logMessage(LOGMAJOR, " - Min Avionics Time: " + GOAL_AVIONICS_TIME).
	logMessage(LOGMAJOR, " - Target Apo Reached During Flight: " + TARGET_APO_REACHED).
	logMessage(LOGMAJOR,"PROGRAM COMPLETE").
	until false {
		WAIT 1.
	}
}

function logTelemetry {
	if (not CFG_LOGGING_ENABLED or MYLOGFILE = "") {
		return.
	}

	set LOGGED_PITCH to currentPitchDeg(MY_VESSEL).
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

