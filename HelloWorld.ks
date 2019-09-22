lock throttle to 1.
print "BANZAI".
lock steering to up.
stage.
wait 2.
stage.
UNTIL SHIP:MAXTHRUST = 0 {
    WAIT 1.
    PRINT "NOMINAL".
}
