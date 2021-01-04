global lz1 to latlng(28.608387739597, -80.5997479624891).
global lz1alt to 145.7.
// global lz1 to latlng(28.608387575067, -80.6063059604572).
// global lz1alt to 248.5.
// global lz1 to latlng(28.6087537525742, -80.6110702149068).
// global lz1alt to 154.5.


// global relativeOvershoots to list(0.02, 0.02).
global relativeOvershoots to list(0, 0).
global boostbackThrottle to 0.6.
global reentryThrottle to 0.8.

// ~~~~~~~~~~~~~~~~~~~~~ return to launch site values ~~~~~~~~~~~~~~~~~~~~~
global reentryAltitude to 55000.
global reentryCutoffSpeed to 700.

global landingAltitude to 4000.
//global landingAltitude to 3000.
global reentryAoA to 45.
global boostbackPitch to 10.


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Boostback functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

function boostbackComplete {
  declare parameter targetPos.

  if not addons:tr:hasImpact { return false. }
  return distance(targetWithOvershoot(targetPos, relativeOvershoots[0]), addons:tr:impactpos) < 0.01.
}

function boostbackClose {
  declare parameter targetPos.

  if not addons:tr:hasImpact { return false. }
  return distance(targetWithOvershoot(targetPos, relativeOvershoots[0]), addons:tr:impactpos) < 0.25.
}

function boostbackSteering {
  declare parameter targetLandingPos.
  if not addons:tr:hasImpact { return heading(270, boostbackPitch). }
  local targetPos to targetWithOvershoot(targetLandingPos, relativeOvershoots[0]).
  local impactPos to addons:tr:impactpos.

  local difLat to targetPos:lat - impactPos:lat.
  local difLng to targetPos:lng - impactPos:lng.

  set yaw to arcsin(difLat / distance(impactPos, targetPos)).
  return heading(270 + yaw, boostbackPitch).
}

function coastAndTurn {
  // Small RCS bursts until a few seconds before reentry burn
  until ship:altitude <= reentryAltitude * 1.2 {
    rcs off.
    wait 5.
    rcs on.
    wait 1.
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Reentry functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

function reentryBurnComplete {
  return ship:velocity:surface:mag <= reentryCutoffSpeed.
}

function centerReentryBurnComplete {
  return ship:velocity:surface:mag <= centerReentryCutoffSpeed.
}

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

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Atmospheric descent functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Landing functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

function landingSteering {
  declare parameter targetLandingPos.
  declare parameter height.

  if abs(ship:verticalspeed) >= 300 {
    return lookdirup(-velocity:surface, vcrs(V(0, 1, 0), ship:body:position)).
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

// reach 0 velocity at 0 altitude, standard hoverslam
function defaultLandingThrottle {
  declare parameter height.
  declare parameter minThrottle.

  return landingThrottle(0, height, 0, minThrottle).
}

function landingThrottle {
  declare parameter targetSpeed.
  declare parameter height.
  declare parameter targetHeight.
  declare parameter minThrottle.

  // that's not perfect since the mass of the craft changes during the landing burn, but it's good enough to just use the current mass
  local calcMass to ship:mass.
  local aimSpeed to abs(ship:verticalspeed) - targetSpeed.
  local aimHeight to height - targetHeight.

  // we have overshot our target, just return full throttle
  if aimHeight < 0 {
    return 1.0.
  }
  // we ran out of fuel
  if ship:maxthrustat(1.0) = 0 {
    return 0.0.
  }

  // optimal percentage of thrust to reach [targetSpeed] at [targetHeight]
  local totalThrottle to calcMass * ((abs(ship:verticalspeed)^2 - targetSpeed^2) / (2 * aimHeight) + 9.81) / ship:maxthrustat(1.0).

  // transform totalThrottle to define a throttle in the range [minThrottle, 1]
  local throttleCapped to max(min(totalThrottle, 1.0), minThrottle + 0.01).
  local transformMultiplier to 1 / (1 - minThrottle).

  return (throttleCapped - minThrottle) * transformMultiplier.
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// General utility functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

set lastPos to latlng(0, 0).
set lastTime to time:seconds.

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

function distance {
  declare parameter pos1, pos2.
  local dif to V(pos1:lat - pos2:lat, pos1:lng - pos2:lng, 0).
  return dif:mag.
}
