WAIT UNTIL SHIP:UNPACKED.
// CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
// set terminal:width to 60.
// set terminal:height to 60.
CLEARSCREEN.

// ensure in format YYYY[XP|F9|FH|SS|SH]NNN[T|S|D|I]
// XP: Experimental
// F9: Falcon 9
// FH: Falcon Heavy
// SS: Starship
// SH: Starship Hopper
// T: Test Flight
// S: Suborbital
// O: Orbital
// I: Interplanetary
global MISSION_ID is "2026EX002". 

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
  // Run directly from archive - avoids local volume memory limits
  runoncepath("0:/common/Utils"). // utils are always available
  runoncepath("0:/missions/" + MISSION_ID). // run the mission
  print "GO FLIGHT".
    // ## END CONFIGURE MISSION ##
}

if ch = "A" { // ABORT
  print "FLIGHT ABORTED".
}
