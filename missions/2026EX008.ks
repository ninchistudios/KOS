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
print "# MISSION: 2026EX008                                     #".
print "# Contract: Suborbital Recovered Science                 #".
print "#                                                        #".
print "# Mission Objective:                                     #".
print "# - launch to suborbital altitude and recover science    #".
print "##########################################################".
print " ".

// Staging Configuration:
// Stage 0: upper stack engine fairing, upper stack separator, upper stack engine (non-throttleable, gimbaled, vacuum optimised)
// Stage 1: booster separators at booster burnout
// Stage 2: gantry and solid boosters ignite
// Stage 3: main engine (non-throttleable, gimbaled, surface optimised)


// CONFIGURE FLIGHT
// ------------------------------------------------------------
// VEHICLE AND PAD GEOMETRY
local CFG_AGL_BARE is 17.7.

// ASCENT TARGETS
local CFG_CUT_APO is 215000.
local CFG_CIRC_ALT is 180000.
local CFG_CIRC_MIN_PERI is 160000.
local CFG_TARGET_HEADING is 90.
local CFG_INITIAL_PITCH is 89.0.
local CFG_PITCH_START_ALT is 1000.

// DESCENT PROFILE
local CFG_CHUTE_STAGE_ALT is 14000.
local CFG_DESCENT_PITCH is 90.

// COUNTDOWN AND ENGINE START SAFETY
local CFG_TCOUNT is 7.
local CFG_TIGNITE is 4.
local CFG_TTHROTTLE_MIN is 0.32.
local CFG_TTHROTTLE_GO is 1.
local CFG_TGANTRY is 0.
local CFG_ENGINE_START_SPOOL_TIME is 2.45.
local CFG_ENGINE_START_TMIN_EXPECTED_FRAC is 0.95.
local CFG_ENGINE_START_FULL_EXPECTED_FRAC is 0.90.

// BOOSTER STAGING
local CFG_HAS_LAUNCH_BOOSTERS is true.
local CFG_BOOSTER_STAGE_DELAY is 0.5.

// WARP, TELEMETRY, LOGGING
local CFG_DO_WARP is false.
local CFG_WARP_SPEED is 3.
local CFG_TELEMETRY_ENABLED is true.
local CFG_LOGGING_ENABLED is false.
local CFG_SRB_NAMES is list("Nike").

// PAYLOAD DEPLOYMENT SAFETY
local CFG_DEPLOY_ACCEL_SAFE is true.
local CFG_DEPLOY_ACCEL_SAFE_ALT is 100000.
local CFG_DEPLOY_ACCEL_SAFE_Q is 0.

// MISSION SUCCESS CRITERIA
local CFG_ENGINE_FUEL_RESOURCE is "Ethanol90".
local CFG_ENGINE_BURNOUT_REMAINING_FRAC is 0.005.
local CFG_MISSION_MIN_APO is 150000.
local CFG_MISSION_MIN_AVIONICS_TIME is 50.
local CFG_MISSION_MAX_APO_ERROR is 10000.
local CFG_CIRC_PID_P is 0.004.
local CFG_CIRC_PID_I is 0.0003.
local CFG_CIRC_PID_D is 0.025.
local CFG_CIRC_PID_OUT_MIN is -0.08.
local CFG_CIRC_PID_OUT_MAX is 0.08.
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
local CIRC_GUIDANCE_ACTIVE is false.
local CIRC_GUIDANCE_FLIP is false.
local CIRC_ENGINE_CUTOFF_DONE is false.
global CPPID is PIDLOOP(CFG_CIRC_PID_P, CFG_CIRC_PID_I, CFG_CIRC_PID_D, CFG_CIRC_PID_OUT_MIN, CFG_CIRC_PID_OUT_MAX).
global CIRC_PITCH is 0.

local GOAL_MIN_APO is false.
local GOAL_TARGET_APO is false.
local GOAL_SUBORBITAL is true.
local GOAL_AVIONICS_TIME is false.
local ENGINE_FUEL_AT_LAUNCH is 0.
local BOOSTERS_SEPARATED is not CFG_HAS_LAUNCH_BOOSTERS.

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

			// first thrust-drop with boosters attached is treated as booster burnout/separation.
			when (CFG_HAS_LAUNCH_BOOSTERS and not BOOSTERS_SEPARATED and stageNeeded(MY_VESSEL)) then {
				doBoosterSeparation().
			}

			// burnout or risking unplanned orbit - cut engines
			when ((BOOSTERS_SEPARATED and stageNeeded(MY_VESSEL)) or MY_VESSEL:PERIAPSIS > 0 or (CFG_CUT_APO <> -1 and MY_VESSEL:APOAPSIS > CFG_CUT_APO)) then {
				doMECO().

				// ballistic coast - wait for impact
				when (MY_VESSEL:STATUS = "LANDED" or MY_VESSEL:STATUS = "SPLASHED") then {
					doTouchDown().
				}
			}
		}
	}
}

function doBoosterSeparation {
	if BOOSTERS_SEPARATED {
		return.
	}
	logMessage(LOGMAJOR,"BOOSTER BURNOUT - STAGE 1 SEP").
	doSafeStage().
	set BOOSTERS_SEPARATED to true.
	doStageDelay(CFG_BOOSTER_STAGE_DELAY).
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
	logMessage(LOGADVISORY, "Target Heading: " + CFG_TARGET_HEADING + " / Initial Pitch: " + CFG_INITIAL_PITCH).
	logMessage(LOGADVISORY, "Pitch Program Alt: " + CFG_PITCH_START_ALT/1000 + "km").

	if (CFG_LOGGING_ENABLED and MYLOGFILE <> "") {
		log logpid() + ",,,," to MYLOGFILE.
		log "MET,ALT,PITCH,Q,APO,DOWNRANGE" to MYLOGFILE.
	}

	if not doCountdownWithEngineStartScrub(
		CFG_TCOUNT,
		CFG_TIGNITE,
		CFG_TTHROTTLE_MIN,
		CFG_TGANTRY,
		CFG_TTHROTTLE_GO,
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
		logMessage(LOGADVISORY,"Pitch Program - HDG " + CFG_TARGET_HEADING + " / CIRC HOLD " + ROUND(CFG_CIRC_ALT/1000,0) + "km").
		lock STEERING to heading(CFG_TARGET_HEADING, missionGuidancePitch(MY_VESSEL)).
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
	local fuelNow is getResourceAmount(CFG_ENGINE_FUEL_RESOURCE).
	local fuelLimit is ENGINE_FUEL_AT_LAUNCH * CFG_ENGINE_BURNOUT_REMAINING_FRAC.
	logMessage(LOGADVISORY,"BURNOUT MET+" + met + "s / DOWNRANGE " + ROUND(DOWNRANGE/1000,1) + "km").
	logMessage(LOGADVISORY,"FUEL " + CFG_ENGINE_FUEL_RESOURCE + ROUND(fuelNow,4) + " / " + ROUND(fuelLimit,4)).

	if (ENGINE_FUEL_AT_LAUNCH <= 0 or fuelNow > fuelLimit or MY_VESSEL:APOAPSIS < CFG_MISSION_MIN_APO) {
		logMessage(LOGERROR,"ENGINE OUT OF NOMINAL").
	} else {
		logMessage(LOGADVISORY,"ENGINE BURN NOMINAL").
	}

	lock STEERING to heading(CFG_TARGET_HEADING, missionGuidancePitch(MY_VESSEL)).
	logMessage(LOGADVISORY,"UPPER STAGE GUIDANCE ACTIVE").

	// Vehicle config: Stage 0 is upper stack separation/engine; execute immediately at MECO.
	lock THROTTLE to CFG_TTHROTTLE_GO.
	logMessage(LOGADVISORY,"THROTTLE GO FOR STAGE 0 IGNITION").
	logMessage(LOGADVISORY,"STAGE 0 HANDOFF").
	doSafeStage().
	if MY_VESSEL:AVAILABLETHRUST <= 0 {
		logMessage(LOGERROR,"STAGE 0 ENGINE NO THRUST AFTER HANDOFF").
	} else {
		logMessage(LOGADVISORY,"STAGE 0 ENGINE LIT").
	}
	set ACCEL_SAFE_DEPLOYED to true.

	when (not CIRC_ENGINE_CUTOFF_DONE and CIRC_GUIDANCE_ACTIVE and MY_VESSEL:PERIAPSIS >= CFG_CIRC_MIN_PERI and ETA:APOAPSIS > ETA:PERIAPSIS) then {
		set CIRC_ENGINE_CUTOFF_DONE to true.
		lock THROTTLE to 0.
		set CIRC_GUIDANCE_ACTIVE to false.
		logMessage(LOGMAJOR,"CIRC COMPLETE - ENGINE CUTOFF").
		logMessage(LOGADVISORY,"CUTOFF APO " + ROUND(MY_VESSEL:APOAPSIS/1000,1) + "km / PERI " + ROUND(MY_VESSEL:PERIAPSIS/1000,1) + "km").
	}

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

		local ENGINES is list().
		local ENGINE_ROW is 15.
		local ENGINE_COUNT is 0.
		local TOTAL_THRUST_KN is 0.

		list engines in ENGINES.
		for EN in ENGINES {
			if not isSrbEngine(EN) {
				set ENGINE_COUNT to ENGINE_COUNT + 1.
				set TOTAL_THRUST_KN to TOTAL_THRUST_KN + EN:THRUST.
				if ENGINE_ROW > 8 {
					logConsole(LOGTELEMETRY,"E" + ENGINE_COUNT,ROUND(EN:THRUST,1),ENGINE_ROW).
					set ENGINE_ROW to ENGINE_ROW - 1.
				}
			}
		}

		logConsole(LOGTELEMETRY,"THRST",ROUND(TOTAL_THRUST_KN,1),7).
		if ENGINE_COUNT > 7 {
			logConsole(LOGTELEMETRY,"ENG HIDE",ENGINE_COUNT - 7,8).
		}
	}
}

function isSrbEngine {
	parameter EN.
	local ENAME is EN:NAME.
	for TOKEN in CFG_SRB_NAMES {
		if ENAME:contains(TOKEN) {
			return true.
		}
	}
	return false.
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
	set ENGINE_FUEL_AT_LAUNCH to getResourceAmount(CFG_ENGINE_FUEL_RESOURCE).
	logMessage(LOGADVISORY,"Engine Fuel @ Launch (" + CFG_ENGINE_FUEL_RESOURCE + "): " + ROUND(ENGINE_FUEL_AT_LAUNCH,4)).
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

function missionGuidancePitch {
	parameter vess.

	if (not CIRC_GUIDANCE_ACTIVE and vess:APOAPSIS >= CFG_CIRC_ALT) {
		set CIRC_GUIDANCE_ACTIVE to true.
		set CIRC_GUIDANCE_FLIP to false.
		set CIRC_PITCH to currentPitchDeg(vess).
		initCircPID(CFG_CIRC_ALT).
		logMessage(LOGMAJOR,"CIRC GUIDANCE HOLD " + ROUND(CFG_CIRC_ALT/1000,0) + "KM APO").
	}

	if (CIRC_GUIDANCE_ACTIVE and not CIRC_GUIDANCE_FLIP and ETA:APOAPSIS > ETA:PERIAPSIS) {
		set CIRC_GUIDANCE_FLIP to true.
		// logMessage(LOGADVISORY,"PAST APOAPSIS - INVERTING CIRC GUIDANCE").
	}

	if CIRC_GUIDANCE_ACTIVE {
		return circPitch(vess, CIRC_GUIDANCE_FLIP).
	}

	return ascentPitch(vess, CFG_MISSION_MIN_APO, CFG_INITIAL_PITCH).
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

