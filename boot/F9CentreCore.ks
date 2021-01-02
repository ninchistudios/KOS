core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
set terminal:width to 60.
set terminal:height to 40.
CLEARSCREEN.

print "#######################################".
print "# KERPERIUM F9 CENTRE CORE BOOTLOADER #".
print "#######################################".
print " ".
print "Go Flight: [F]".
print "Abort: [A]".

set validInput to false.
until validInput {
  set ch to terminal:input:getchar().
  if (ch = "F" or ch = "A") set validInput to true.
}

if ch = "F" { // GO FLIGHT
  print "# GO FLIGHT #".
  // ## CONFIGURE MISSION ##
  copypath("0:/F9/HoverSlamTest1", "").
  runoncepath("HoverSlamTest1").
    // ## END CONFIGURE MISSION ##
}

if ch = "A" { // ABORT
  print "# FLIGHT ABORTED #".
}
