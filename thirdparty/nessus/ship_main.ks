run once ascent.
run once bfr_util.

// ~~~~~~~~~~~~~~~~~ Startup ~~~~~~~~~~~~~~~~
clearscreen.
print "Init".
set thrott to 1.0.
lock throttle to thrott.
wait 3.

// ~~~~~~~~~~~~~~~~~ Ignition ~~~~~~~~~~~~~~~~~~~
controlFromBooster().
stage.
wait 7.
set pitch to 90.
lock steering to heading(90, pitch).

// ~~~~~~~~~~~~~~~~~ Liftoff ~~~~~~~~~~~~~~~~
stage.
wait 2.
until readyForBoosterSep() {
  //ascent until booster separation
  set pitch to pitchForAltitude(ship:altitude).
  wait 0.2.
}

// ~~~~~~~~~~~~~~~~~ Booster Separation ~~~~~~~~~~~~~~~~~~~~~~

set thrott to 0.
wait 1.
unlock steering.
stage.
rcs on.
set ship:control:fore to 1.0.

// wait for the second stage to move away from the center core
wait 2.
//set thrott to 0.5.
lock steering to heading(90, 25).
wait 1.

set ship:control:fore to 0.0.
set thrott to 0.2.
wait 3.
set thrott to 1.0.

wait 10.
