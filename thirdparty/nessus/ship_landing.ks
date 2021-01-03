run landing.
clearscreen.

toggle ag2.

// global shipLandingPosition to latlng(28.6084095848539, -80.596476358078).
// global shipLandingAlt to 106.
global shipLandingPosition to latlng(28.608387575067, -80.6063059604572).
global shipLandingAlt to 227.

lock height to ship:altitude - shipLandingAlt.

print "Starting".
rcs on.

set thrott to 0.0.
lock throttle to thrott.

unlock steering.
set ship:control:pitch to 1.0.
wait 3.3.
set ship:control:pitch to 0.0.

set steer to -ship:velocity:surface.
lock steering to steer.
wait 1.
toggle ag1.
set thrott to 0.1.
wait 2.

//until ship:altitude <= shipLandingAlt {
until ship:verticalspeed >= -0.1 {
  set steer to landingSteering(shipLandingPosition, height).
  set thrott to defaultLandingThrottle(height, 0.2).
  wait 0.05.
}

set thrott to 0.0.

wait 1.
