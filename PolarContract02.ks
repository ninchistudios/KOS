@LAZYGLOBAL OFF.

print " ".
print "#####################################################".
print "# POLAR CONTRACT 02 (PROBODOBODYNE) ORBITAL PROGRAM #".
print "#####################################################".
print " ".

// CONFIGURE LAUNCH
local TGT_APO is 90000. // target orbit to be circularised
local TGT_INCL is 0. // target orbital inclination
local TCOUNT is 3. // T-Minus countdown

// RUN
doSetup().
doMain().
doFinalise().

// runs once
function doSetup {
  global MY_VESSEL is SHIP.
  // surface key flight data
  print "Target Orbit: " + TGT_APO + "m".
  print "Target Orbital Inclination: " + TGT_INCL + " degrees".
  doCountdown(TCOUNT).
}

// loops while program executing
function doMain {
  // stage until we have available thrust
  // stage any time available thrust is 0 (flamout) or drops sharply (booster)
  // short delay after 0 thrust staging before throttle comes up (prevent boom)
  // go (almost) straight up until we clear the tower
  // pitch to 87' by 1k
  // pitch to 45' by 10k
  // pitch to 10' by 40k?
  // maintain steering to achieve target orbital inclination
  // go ballistic once apo is at target orbit
  // deploy fairings once out of atmo
  // create maneuver to circularise, via genetic algo
  // calc burn time of maneuver
  // execute circ burn
}

function doCountdown {
  local parameter t.
  print "# T MINUS".
  from {local c is t.} until c = 0 step {set c to c - 1.} do {
    print "# ... " + c.
    WAIT 1.
  }
}

// run last
function doFinalise {
  lock THROTTLE to 0.
  set MY_VESSEL:control:neutralize to true.
  print "### PROGRAM COMPLETE ###".
}
