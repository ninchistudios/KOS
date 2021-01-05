@LAZYGLOBAL OFF.

// LANDING ZONES
// KSC LZ1: Lat -000.032492 Long -074.663904 0 AGL 67m AMSL
// KSC LZ2: Lat -000.032570 Long -074.642213 0 AGL 67m AMSL
// KSC FIWLT: Lat -000.149711 Long -074.013484 AGL 741.7m AMSL 0
// KSC OCISLY: Lat 0*08'59" S Long 74*00'48" W AGL 744.8m AMSL 2.49m


global KSCLZ1 to latlng(-0.032492, -74.663904). // 0 AGL 67m AMSL
global KSCLZ2 to latlng(-0.032570, -74.642213). // 0 AGL 67m AMSL
global KSCFIWLT to latlng(-0.149711, -74.013484).// 741.7 AGL 0 AMSL
global KSCOCISLY to latlng(decimalDegrees(0,8,59,"S"),decimalDegrees(74,0,48,"W")).
global relativeOvershoots to list(0.02, 0.02).
global reentryAltitude to 55000.
global reentryCutoffSpeed to 700.
global landingAltitude to 4000.
global reentryAoA to 5.
global boostbackPitch to 10.
local lastPos to latlng(0, 0).
local lastTime to time:seconds.

// original code by nessus
function boostbackComplete {
  declare parameter targetPos.
  if not addons:tr:hasImpact { return false. }
  return distance(targetWithOvershoot(targetPos, relativeOvershoots[0]), addons:tr:impactpos) < 0.01.
}

// original code by nessus
function boostbackClose {
  declare parameter targetPos.
  if not addons:tr:hasImpact { return false. }
  return distance(targetWithOvershoot(targetPos, relativeOvershoots[0]), addons:tr:impactpos) < 1.
}

// original code by nessus
// param targetLandingPos: a latlng for the landing target
function boostbackSteering {
  declare parameter targetLandingPos,maxPitch.
  if not addons:tr:hasImpact { return heading(270, boostbackPitch). }
  local targetPos to targetWithOvershoot(targetLandingPos, relativeOvershoots[0]).
  local impactPos to addons:tr:impactpos.

  local difLat to targetPos:lat - impactPos:lat.
  local difLng to targetPos:lng - impactPos:lng.

  local yaw to arcsin(difLat / distance(impactPos, targetPos)).
  return heading(270 + yaw, maxPitch).
}

// original code by nessus
function reentryBurnSteering {
  declare parameter targetLandingPos.
  local targetPos to targetWithOvershoot(targetLandingPos, relativeOvershoots[1]).
  local impactPos to addons:tr:impactpos.

  local difLat to targetPos:lat - impactPos:lat.
  local difLng to targetPos:lng - impactPos:lng.

  local pitch to max(min(difLng * 2000, reentryAoA), -reentryAoA).
  local yaw to max(min(difLat * 2000, reentryAoA), -reentryAoA).

  local dirRetro to lookdirup(-velocity:surface, vcrs(V(0, 1, 0), ship:body:position)).
  return dirRetro + R(-yaw, -pitch, 0).
}

// original code by nessus
function atmosphericDescentSteering {
  declare parameter targetLandingPos.
  //magic numer alert: the following numers and calculations were found by trial and error and are responsible for the shape of the descent trajectory
  //local mult to 0.4 - (ship:altitude / 40000).

  local mult to 0.4 + (0.3 * (ship:altitude / 30000)).
  local timeToImpact to mult * ship:altitude / abs(ship:verticalspeed).

  //local timeToImpact to 0.
  //if ship:altitude >= 10300 {
  //  set timeToImpact to 0.00149 * ship:altitude + 30.97.
  //} else {
  //  set timeToImpact to 0.0026 * ship:altitude + 19.59.
  //}
  //print timeToImpact at (0, 0).

  local impact to positionWithSpeedOvershoot(timeToImpact).
  local impactToTarget to latlng(targetLandingPos:lat - impact:lat,
                                 targetLandingPos:lng - impact:lng).
  //local mult2 to 6 - (6 * ship:altitude / 30000).
  //local mult2 to 3 + (3 * ship:altitude / 30000).
  local mult2 to 8.
  local targetPos to latlng(targetLandingPos:lat + mult2 * impactToTarget:lat,
                            targetLandingPos:lng + mult2 * impactToTarget:lng).

  local targetVec to targetPos:position.
  local dirTarget to lookdirup(-targetVec, vcrs(V(0, 1, 0), ship:body:position)).
  return dirTarget.
}

// original code by nessus
function landingSteering {
  declare parameter targetLandingPos.
  declare parameter height.

  if abs(ship:verticalspeed) >= 300 {
    // return lookdirup(-velocity:surface, vcrs(V(0, 1, 0), ship:body:position)).
  }
  if abs(ship:verticalspeed) <= 15{//15 {
    return lookdirup(-ship:body:position, vcrs(V(0, 1, 0), ship:body:position)).
  }

  //local timeToImpact to 1.//height / abs(ship:verticalspeed).
  local timeToImpact to 0.6 * height / abs(ship:verticalspeed).
  local impact to positionWithSpeedOvershoot(timeToImpact).
  local targetToImpact to latlng(impact:lat - targetLandingPos:lat,
                                 impact:lng - targetLandingPos:lng).

  // local mult to 7.
  // if (height < 300) {
  //   set mult to 1 + (6 * height / 300).
  // }
  local mult to 5.
  if (height < 300) {
    set mult to 1 + (4 * height / 300).
  }
  local targetPos to latlng(targetLandingPos:lat + mult * targetToImpact:lat,
                            targetLandingPos:lng + mult * targetToImpact:lng).

  local targetVec to targetPos:position.
  local dirTarget to lookdirup(-targetVec, vcrs(V(0, 1, 0), ship:body:position)).
  return dirTarget.
}

// original code by nessus
function targetWithOvershoot {
  declare parameter targetLandingPos.
  declare parameter relativeOvershoot.

  local nowPos to ship:geoposition.
  local geoDiff to latlng(targetLandingPos:lat - nowPos:lat,
                          targetLandingPos:lng - nowPos:lng).
  local targetPos to latlng(targetLandingPos:lat + (geoDiff:lat * relativeOvershoot),
                            targetLandingPos:lng + (geoDiff:lng * relativeOvershoot)).

  return targetPos.
}

// original code by nessus
function positionWithSpeedOvershoot {
  declare parameter speedMultiplier.

  local deltaT to time:seconds - lastTime.
  local deltaPos to latlng(ship:geoposition:lat - lastPos:lat,
                           ship:geoposition:lng - lastPos:lng).
  local geoSpeed to latlng(deltaPos:lat / deltaT, deltaPos:lng / deltaT).

  set lastPos to ship:geoposition.
  set lastTime to time:seconds.

  return latlng(lastPos:lat + (speedMultiplier * geoSpeed:lat),
                lastPos:lng + (speedMultiplier * geoSpeed:lng)).
}

// original code by nessus
function distanceFromSteering {
  local dir1 to ship:facing.
  local dir2 to steering.

  function minDiff { declare parameter diff.
    return min(min(abs(diff), abs(diff - 360)), abs(diff + 360)).
  }

  set pitchDiff to minDiff(dir1:pitch - dir2:pitch).
  set yawDiff to minDiff(dir1:yaw - dir2:yaw).

  return V(pitchDiff, yawDiff, 0):mag.
}

// original code by nessus
function distance {
  declare parameter pos1, pos2.
  local dif to V(pos1:lat - pos2:lat, pos1:lng - pos2:lng, 0).
  return dif:mag.
}

//
// param AGL :
// param HAGL :
// param Tmin :
// returns
function hoverThrottle {
  parameter AGL,HAGL,Tmin.
  // thrott = TWRd x Fg / AVAILABLETHRUST
  // TWRd = (0.00012 * dV^3) + (0.000514286 * dV^2 + (0.003 * dV) +0.998286)
  // dV = (0.0732601 * dH^3) - (17.326 * dH)
  local dH is AGL - HAGL. // delta between desired height and actual height
  local dV is desiredVv(dH). // desired surface velocity based on dH
  local TWRd is desiredTWR(dV, ship:verticalspeed). // desired TWR
  // local thrott is Fg * Tf / COS(vAngle) / AVAILABLETHRUST.
  local vthrott is TWRd * Fg / max(1,AVAILABLETHRUST). // throttle assuming vertical
  local thrott is vthrott / COS(vAngle).
  if (thrott < Tmin) {
    set thrott to Tmin.
    // TODO if we're still accelerating up we need to shutdown some engines
  } else if (thrott > 1) {
    set thrott to 1.
  }
  return thrott.
}

//
// param LAUNCH_AGL :
// param AGL_TWEAK :
// returns
function distanceToGround {
  parameter LAUNCH_AGL,AGL_TWEAK.
  return altitude - body:geopositionOf(ship:position):terrainHeight - LAUNCH_AGL - AGL_TWEAK.
}

//
function stoppingDistance {
  local grav is constant():g * (body:mass / body:radius^2).
  local maxDeceleration is (ship:availableThrust / ship:mass) - grav.
  return ship:verticalSpeed^2 / (2 * maxDeceleration).
}

//
// param AGL :
// returns
function predictedRadalt {
  parameter AGL.
  local s0 is AGL. // AGL of the nozzles
  local u is ship:verticalspeed. // the current velocity, -ve up
  local a is ship:sensors:acc:y. // this appears to be very unreliable
  // t is the time to v = 0 under the current a: v=u+at
  local t is (0 - u) / a.
  // distance to peak: s=ut+1/2at^2
  local s is (u * t) + (0.5 * a * (t^2)).
  set PREDICTED to s + s0. // TODO is this correct?
  return PREDICTED.
}


// basic function to calculate the desired vertical speed based on the distance from the target height
// param dH1 :
// returns
function desiredVv {
  parameter dH1.
  return max(-20,min(10,(-0.00000533333 * dH1^3) + (0.000000000000000000243714 * dH1^2) - (.0466667 * dH1))).
}

// basic function to calculate the desired TWR based on the desired vertical speed and current vertical speed
// param dv0 :
// param v0 :
// returns
function desiredTWR {
  parameter dv0,v0.
  local dv1 is dv0 - v0.
  // return max(0.8,min(1.3,(0.00012 * dV1^3) + (0.000514286 * dV1^2 + (0.003 * dV1) +0.998286))).
  return max(0,min(1.3,(0.038118 + (0.961882 * (constant:e ^ (0.0770805 * dv1)))))).
}
