// THIS SCRIPT SHOULD HAVE NO OUTBOUND DEPENDENCIES OTHER THAN UTILS
// IF NEEDED, LOAD IN BOOT FILE AFTER UTILS AND BEFORE MISSION SCRIPT

@LAZYGLOBAL OFF.

// GLOBALS - launch-domain state
local LAST_T is 0.
local TICK is 0.
local HIGHEST_Q is 0.
local NO_STAGE_BEFORE is 0.

local CPp is 0.04.
local CPi is 0.01.
local CPd is 0.15.
local CLAMP_POS_PITCH is 15.
local CLAMP_NEG_PITCH is -89.

function neutraliseRoll {
	SET STEERINGMANAGER:ROLLPID:KP TO 0.
	SET STEERINGMANAGER:ROLLPID:KI TO 0.
}

function logpid {
	return "P:" + CPp + " I:" + CPi + " D:" + CPd + " +C:" + CLAMP_POS_PITCH + " -C:" + CLAMP_NEG_PITCH.
}

function towerThrottle {
	return 1.
}

function hoverSteering {
	return up.
}

// returns the true radalt of the base of the vehicle, adjusted for height of the control module
function baseRadalt {
	parameter bareRadAlt.
	return MAX(0.00001,((ALTITUDE-GEOPOSITION:TERRAINHEIGHT)-bareRadAlt)).
}

// control the throttle to reduce Q if needed
function ascentThrottle {
	parameter Tmin, Tmax, qMax, iQ, limitQ.
	if not limitQ {
		return Tmax.
	}
	if iQ <= qMax {
		return Tmax.
	}
	local qBand is MAX(0.1, qMax * 0.25).
	local overLimit is iQ - qMax.
	local scale is MIN(1, overLimit / qBand).
	return MAX(Tmin, (Tmax - (scale * (Tmax - Tmin)))).
}

function ascentHeading {
	parameter tgt_incl.
	return tgt_incl.
}

// ascent on a logarithmic path
function ascentPitch {
	parameter vess, targetApo.
	if vess:APOAPSIS <= 0 { return 89.9. }
	local apoTarget is MAX(1, targetApo).
	local c is 1 / apoTarget.
	local tp is min(89.9,-30 * ln(vess:APOAPSIS * c)).
	return max(0, tp).
}

// vacuum accel-safe deployments (e.g. fairings) should be set to AG8
function deployAccelSafe {
	AG8 ON.
	logMessage(LOGADVISORY,"ACCEL-SAFE MODULES DEPLOYED").
}

// vacuum orbit-safe deployments (e.g. science) should be set to AG9
function deployOrbitSafe {
	AG9 ON.
	logMessage(LOGADVISORY,"ORBIT-SAFE MODULES DEPLOYED").
}

// stage with a delay until ready
function doSafeStage {
	wait until STAGE:READY.
	STAGE.
}

// when updated with the latest Q, returns the highest Q reported
function topQ {
	parameter latestQ.
	if latestQ > HIGHEST_Q {
		set HIGHEST_Q to latestQ.
	}
	return HIGHEST_Q.
}

// usually used to give ullage boosters a chance to do their thing
function doStageDelay {
	parameter delay.
	set NO_STAGE_BEFORE to TIME:SECONDS + delay.
	set LAST_T to 0.
}

// have we experienced a drop in available thrust, and thus staging needed?
function stageNeeded {
	parameter vess.
	if TIME:SECONDS < NO_STAGE_BEFORE {
		return false.
	}
	local t is vess:AVAILABLETHRUST.
	set TICK to TICK + 1.
	if (t = 0 or t < (LAST_T - 10)) {
		set LAST_T to t.
		return true.
	} else {
		set LAST_T to t.
		return false.
	}
}

// T-minus countdown
function doCountdown {
	local parameter t,i,Tmin,gt.
	doCountdownWithThrottle(t,i,Tmin,gt,-1).
}

// T-minus countdown with optional full-throttle handoff point
function doCountdownWithThrottle {
	local parameter t,i,Tmin,gt,tf.
	local throttlePrimed is false.
	logMessage(LOGADVISORY,"T MINUS").
	from {local c is t.} until c = -1 step {set c to c - 1.} do {
		WAIT 1.
		logMessage(LOGADVISORY," ... " + c).
		if (i <> -1 and not throttlePrimed and c = (i + 1)) {
			lock throttle to Tmin.
			logMessage(LOGADVISORY,"THROTTLE PRIMED TO " + Tmin).
			set throttlePrimed to true.
		}
		if (c = gt) {
			logMessage(LOGADVISORY,"GANTRY").
			doSafeStage().
		}
		if (c = i) {
			logMessage(LOGADVISORY,"IGNITION").
			lock throttle to Tmin.
			doSafeStage().
		}
		if (c = tf and (i = -1 or c < i)) {
			logMessage(LOGADVISORY,"FULL THROTTLE").
			lock throttle to 1.
		}
	}
}

// Countdown with thrust spool checks and auto-scrub guard rails.
function doCountdownWithEngineStartScrub {
	parameter t, i, Tmin, gt, tf, vess, engineStartSpoolTime, engineStartTminExpectedFrac, engineStartFullExpectedFrac.
	local throttlePrimed is false.
	local ignitionCommanded is false.
	local spoolChecked is false.
	local ignitionStartTime is -1.
	logMessage(LOGADVISORY,"T MINUS").
	from {local c is t.} until c = -1 step {set c to c - 1.} do {
		WAIT 1.
		logMessage(LOGADVISORY," ... " + c).
		if (i <> -1 and not throttlePrimed and c = (i + 1)) {
			lock throttle to Tmin.
			logMessage(LOGADVISORY,"THROTTLE PRIMED TO " + Tmin).
			set throttlePrimed to true.
		}
		if (c = i) {
			logMessage(LOGADVISORY,"IGNITION").
			lock throttle to Tmin.
			doSafeStage().
			set ignitionCommanded to true.
			set spoolChecked to false.
			set ignitionStartTime to TIME:SECONDS.
		}

		if (ignitionCommanded and not spoolChecked and (TIME:SECONDS - ignitionStartTime) >= engineStartSpoolTime) {
			if not hasExpectedThrustAtThrottle(vess, Tmin, engineStartTminExpectedFrac) {
				doLaunchAutoScrub("MAIN ENGINE FAILED TO REACH TMIN THRUST AFTER SPOOL").
				return false.
			}
			set spoolChecked to true.
		}

		if (c = gt) {
			if (tf = -1 or tf > gt) {
				logMessage(LOGADVISORY,"FULL THROTTLE (PRE-GANTRY)").
				lock throttle to 1.
			}
			if (i <> -1 and ignitionCommanded) {
				if not spoolChecked {
					doLaunchAutoScrub("ENGINE SPOOL CHECK NOT COMPLETE BEFORE GANTRY RELEASE").
					return false.
				}
				if not hasExpectedThrustAtThrottle(vess, 1, engineStartFullExpectedFrac) {
					doLaunchAutoScrub("MAIN ENGINE FAILED TO REACH FULL THRUST BEFORE GANTRY RELEASE").
					return false.
				}
			}
			logMessage(LOGADVISORY,"GANTRY").
			doSafeStage().
		}
		if (c = tf and (i = -1 or c < i)) {
			logMessage(LOGADVISORY,"FULL THROTTLE").
			lock throttle to 1.
		}
	}
	return true.
}

function hasExpectedThrustAtThrottle {
	parameter vess, throttleCmd, expectedFrac.
	local vesselMaxRatedThrust is vess:MAXTHRUST.
	if vesselMaxRatedThrust <= 0 {
		return false.
	}
	return vess:AVAILABLETHRUST >= (vesselMaxRatedThrust * throttleCmd * expectedFrac).
}

function doLaunchAutoScrub {
	parameter reason.
	logMessage(LOGMAJOR,"AUTO SCRUB").
	logMessage(LOGERROR,reason).
	lock throttle to 0.
	set SHIP:CONTROL:ROLL to 0.
	lock steering to up.
	set SHIP:CONTROL:NEUTRALIZE to true.
}

function doGridfins {
	parameter turnOn.
	if (turnOn) {
		AG3 on.
		logMessage(LOGADVISORY,"GRIDFINS OUT").
	} else {
		AG3 off.
		logMessage(LOGADVISORY,"GRIDFINS IN").
	}
}

function currentPitchDeg {
	parameter vess.
	return 90 - VANG(vess:FACING:FOREVECTOR, vess:UP:FOREVECTOR).
}

function clampUnit {
	parameter x.
	if x > 1 { return 1. }
	if x < -1 { return -1. }
	return x.
}

function activateRawSpin {
	parameter tgtHeading, tgtPitch, spinRate.
	lock steering to heading(tgtHeading, tgtPitch).
	set SHIP:CONTROL:ROLL to clampUnit(spinRate).
	logMessage(LOGADVISORY,"RAW SPIN ACTIVE - ROLL " + clampUnit(spinRate)).
}

function missionQThrottle {
	parameter qNow, qLimitKpa, qThrottleMin, qThrottleBandKpa.
	if qNow <= qLimitKpa {
		return 1.
	}

	local overLimit is qNow - qLimitKpa.
	local scale is MIN(1, overLimit / MAX(0.1, qThrottleBandKpa)).
	local limited is 1 - (scale * (1 - qThrottleMin)).
	return MAX(qThrottleMin, limited).
}

function surfaceRelativeSpeed {
	parameter vess.
	local horizontal is vess:GROUNDSPEED.
	local vertical is vess:verticalspeed.
	return SQRT((horizontal * horizontal) + (vertical * vertical)).
}

function launchDownrangeMeters {
	parameter vess, launchGeo.
	return vess:BODY:RADIUS * VANG(
		launchGeo:POSITION - vess:body:position,
		vess:position - vess:body:position
	) * constant:degtorad.
}

