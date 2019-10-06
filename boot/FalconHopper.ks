core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
set terminal:width to 50.
set terminal:height to 60.
CLEARSCREEN.

print "########################".
print "# KERPERIUM BOOTLOADER #".
print "########################".
print " ".
print "Go Flight: [G]".
print "Abort: [A]".

set validInput to false.
until validInput {
  set ch to terminal:input:getchar().
  if (ch = "G" or ch = "A") set validInput to true.
}

if ch = "G" { // GO FLIGHT
  print "# GO FLIGHT #".
  // ## CONFIGURE MISSION ##
  copypath("0:/FalconHopper/FalconHopperI.ks", "").
  copypath("0:/common/Utils.ks", "").
  runoncepath("FalconHopperI").
  // ## END CONFIGURE MISSION ##
}

if ch = "A" { // ABORT
  print "# FLIGHT ABORTED #".
}
