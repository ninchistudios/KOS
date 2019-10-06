clearscreen.

function distance {
  declare parameter pos1, pos2.
  local dif to V(pos1:lat - pos2:lat, pos1:lng - pos2:lng, 0).
  return dif:mag.
}

function normal2D {
  declare parameter vec1.

  if vec1:x = 0 {
    return V(1, 0, 0).
  } else if vec1:y = 0 {
    return V(0, 1, 0).
  }

  return V(1.0 / vec1:x, -1.0 / vec1:y, 0).
}

function solve2D {
  declare parameter a, b, c, d, vecB.

  // calc inverse matrix of (a, b, c, d)
  local m to 1.0 / (a * d - b * c).
  local ai to m * d.
  local bi to m * (-b).
  local ci to m * (-c).
  local di to m * a.

  return V(ai * vecB:x + bi * vecB:y, ci * vecB:x + di * vecB:y, 0).
}

// calculates the position of the closest point between z and the line through p1 and p2 where p1 is 0 and p2 is 1.
function interpolateBetween2D {
  declare parameter p1, p2, z.

  local dir to p2 - p1.
  local w to normal2D(dir).

  return solve2D(dir:x, w:x, dir:y, w:y, z - p1).
}

wait 10.

global shipLandingPosition to latlng(28.608387575067, -80.6063059604572).

global reentryAltitude to 75000.0.
global descentSpeed to -65.
global flipSpeed to 1500.0.

global maxPitch to 35.0.
global landingSteeringAltitude to 32000.

wait until ship:altitude <= 139000.

set steer to heading(0, 0).
//setting up the steering direction: surface prograde with a pitch of [pitch].
lock forward to lookdirup(velocity:surface, -ship:body:position).
lock normVec to vcrs(ship:body:position, ship:velocity:surface).
set pitch to 0.0.
lock steering to angleaxis(pitch, normVec) * forward.

wait until ship:altitude <= reentryAltitude.
rcs on.
toggle ag1.
print "Starting reentry now".
set pitch to 70.0.


wait until ship:verticalspeed >= (descentSpeed - 5).
print "Starting dynamic pitch now".
set pitch to 80.0.
wait 1.
clearscreen.

set vSpeed to ship:verticalspeed.
set vSpeedTime to time:seconds - 0.1.

set dis to 0.
lock dis to distance(addons:tr:impactpos, shipLandingPosition).

//until velocity:surface:mag <= flipSpeed {
//until dis <= 0.06 {
until abs(distance(ship:geoposition, shipLandingPosition) - distance(addons:tr:impactpos, shipLandingPosition)) <= 0.05 {
  set vSpeedN to ship:verticalspeed.
  set vSpeedTimeN to time:seconds.

  set vAcc to (vSpeedN - vSpeed) / (vSpeedTimeN - vSpeedTime).
  print "Vertical accelaration is " + vAcc at (0, 0).
  set vSpeed to vSpeedN.
  set vSpeedTime to vSpeedTimeN.

  if (vSpeed <= descentSpeed) and (vAcc <= 0) {
    set pitch to pitch - 0.1.
    print "Decreasing pitch to " + pitch at (0, 1).
  } else if (vSpeed >= descentSpeed) and (vAcc >= 0) {
    set pitch to pitch + 0.1.
    print "Increasing pitch to " + pitch at (0, 1).
  }
  set pitch to max(min(90.0, pitch), 60.0).

  wait 0.2.
}
//wait 1.
clearscreen.
print "Falling straight down".

set pitch to 90.0.
wait until ship:altitude <= 20000.

unlock forward.
unlock normVec.
unlock steering.

set steer to steering.
lock steering to steer.
// adjust trajectory to hit KSC
print "Dynamic steering".
until ((ship:geoposition:position - shipLandingPosition:position):mag <= 50 or (ship:altitude <= 4000)) {
  local vecPos to V(ship:geoposition:lat, ship:geoposition:lng, 0).
  local vecTarget to V(shipLandingPosition:lat, shipLandingPosition:lng, 0).
  local impact to addons:tr:impactpos.
  local vecImpact to V(impact:lat, impact:lng, 0).

  local t to interpolateBetween2D(vecTarget, vecPos, vecImpact):x.

  local downrangeToTarget to (ship:geoposition:position - shipLandingPosition:position):mag.

  local targetVec to shipLandingPosition:altitudePosition(ship:altitude - ((t - 0.2) * downrangeToTarget * 0.5)).
  local dirTarget to lookdirup(-targetVec, -ship:body:position).

  set steer to dirTarget.
  wait 0.2.
}
print "Close enough".
wait until ship:altitude <= 3500.

unlock forward.
unlock normVec.
unlock steering.

copypath("0:/bfr/landing.ks", "").
copypath("0:/bfr/ship_landing.ks", "").

run ship_landing.ks.
