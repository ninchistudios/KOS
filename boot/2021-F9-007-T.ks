core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
set terminal:width to 60.
set terminal:height to 40.
CLEARSCREEN.

print "#########################".
print "# KERPERIUM BOOTLOADER  #".
print "# MISSION 2021-F9-007-T #".
print "#########################".
print " ".
PRINT "WARNING: ENSURE MECHJEB IS INACTIVE.".
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
  // Late loaded to permit pad editing
  copypath("0:/common/Utils", ""). // utils are always available
  copypath("0:/missions/2021-F9-007-T", ""). // preload the mission
  runoncepath("Utils").
  runoncepath("2021-F9-007-T"). // run the mission
    // ## END CONFIGURE MISSION ##
}

if ch = "A" { // ABORT
  print "# FLIGHT ABORTED #".
}
