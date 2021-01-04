core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
set terminal:width to 60.
set terminal:height to 60.
CLEARSCREEN.

local MISSION_ID is "2021-SH-002-T". /// ensure in format YYYY-[F9|FH|SS|SH]-NNN-[T|D|O]

print "#########################".
print "# KERPERIUM BOOTLOADER  #".
print "# MISSION " + MISSION_ID + " #".
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
  // ## CONFIGURE MISSION ##
  // Late loaded to permit pad editing
  copypath("0:/common/Utils", ""). // utils are always available
  copypath("0:/missions/" + MISSION_ID, ""). // preload the mission
  runoncepath("Utils").
  runoncepath(MISSION_ID). // run the mission
  print "GO FLIGHT".
    // ## END CONFIGURE MISSION ##
}

if ch = "A" { // ABORT
  print "FLIGHT ABORTED".
}
