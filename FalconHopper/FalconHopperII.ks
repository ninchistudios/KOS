@LAZYGLOBAL OFF.

// include utilities once and only once
runoncepath("Utils").

print " ".
print "###################################".
print "# FALCON HOPPER II FLIGHT PROGRAM #".
print "###################################".
print " ".

runoncepath("ascentController").
runoncepath("hoverslamModel").

// -- [ Prelaunch ] -------------
local ascent is ascentController(8000, 90, 10, 3, 90).

// -- [ Launch ] ----------------
// stage.
lock throttle to 1.
stage.

// -- [ Ascent ] ----------------
ascent["passControl"]().

// -- [ Shutdown ] --------------
RCS ON.
AG7 on. // gridfins
AG8 on. // single engine
lock steering to ship:srfretrograde.
wait until ship:verticalspeed < 0. // waiting for descent
when (altitude < 250) then {
  gear on.
}


// -- [ Landing ] ---------------
local hoverslam is hoverslamModel(10).
lock throttle to hoverslam["getThrottle"]().


wait until ship:status = "splashed" or ship:status="landed".
// We've landed!
