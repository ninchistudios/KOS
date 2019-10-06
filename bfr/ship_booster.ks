run once ascent.
run once landing.

wait until stage:LqdMethane > 0.
set lz1 to ship:geoposition.

// ~~~~~~~~~~~~~~ LOG ~~~~~~~~~~~~~~~~~~~
// create("0:flightdata.txt").
// set t to time:seconds.
// when (time:seconds - t) > 0.5 then {
//   set t to time:seconds.
//   log ship:geoposition:lat + "," + ship:geoposition:lng + "," + ship:altitude + "," + abs(ship:verticalspeed) + "," + ship:maxthrustat(0) + "," + throttle to "0:flightdata.txt".
//   preserve.
// }
// ~~~~~~~~~~~~~~ /LOG ~~~~~~~~~~~~~~~~~~~

wait until readyForBoosterSep().

clearscreen.
// ~~~~~~~~~~~~~~ MECO and separation ~~~~~~~~~~~~~~~~~~~
toggle ag2.
wait 4.
run once bfr_util.
wait 1.
unlock steering.
rcs on.
set ship:control:pitch to 1.0.
wait 7.
set ship:control:pitch to 0.0.
rcs off.

wait 4.
// wait 1.
set thrott to boostbackThrottle.
lock throttle to thrott.
set steer to boostbackSteering(ship:geoposition).
lock steering to steer.
wait 1.

// ~~~~~~~~~~~~~~ Boostback Burn start ~~~~~~~~~~~~~~~~~~~
toggle ag1.
wait 1.
wait until distanceFromSteering() < 10.

lock steer to boostbackSteering(lz1).
wait 18.
unlock steer.
toggle ag3.

set thrott to 1.0.
until boostbackClose(lz1) {
  set steer to boostbackSteering(lz1).
  wait 0.05.
}
set thrott to 0.2.
until boostbackComplete(lz1) {
  set steer to boostbackSteering(lz1).
  wait 0.05.
}

toggle ag2.

// ~~~~~~~~~~~~~~ Turn around and coast ~~~~~~~~~~~~~~~~~~~
lock steer to lookdirup(heading(270, 0):forevector, vcrs(V(0, 1, 0), ship:body:position)).
//rcs on.
//wait 8.
//rcs off.
unlock steering.
lock steer to -ship:velocity:surface.

brakes on.

wait until ship:verticalspeed <= 0.
wait until ship:altitude < 130000.
lock steering to steer.
coastAndTurn().
wait until ship:altitude <= reentryAltitude.

// ~~~~~~~~~~~~~~ Reentry Burn start ~~~~~~~~~~~~~~~~~~~
set thrott to reentryThrottle.
toggle ag1.
wait 1.
until reentryBurnComplete() {
  set steer to reentryBurnSteering(lz1).
  wait 0.1.
}
toggle ag2.


// ~~~~~~~~~~~~~~ Atmospheric descent ~~~~~~~~~~~~~~~~~~~
until ship:altitude < landingAltitude {
  set steer to atmosphericDescentSteering(lz1).
  wait 0.1.
}

// ~~~~~~~~~~~~~~ Landing burn start ~~~~~~~~~~~~~~~~~~~
lock height to ship:altitude - lz1alt.
set thrott to 0.1.

toggle ag1.
wait 2.

set multiEngineBurn to true.
//set multiEngineBurn to false.

when abs(ship:verticalspeed) <= 150 then {
  // shutdownInnerBoostEngines().
  set multiEngineBurn to false.

  when abs(ship:verticalspeed) <= 40 then {
    gear on.
  }
}

// until height < 1 or ship:verticalspeed >= -0.2 {
until height < 0 or ship:verticalspeed >= 0 {
  set steer to landingSteering(lz1, height).
  if multiEngineBurn {
    set thrott to landingThrottle(140, height, 1000, 0.2).
  } else {
    set thrott to defaultLandingThrottle(height, 0.2).
    //set thrott to landingThrottle(10, height, 30, 0.2).
  }
  wait 0.05.
}

toggle ag2.
wait 1.
toggle brakes.
wait 4.
print (ship:geoposition:altitudePosition(83) - lz1:altitudePosition(83)):mag at (0, 4).
wait 2.
