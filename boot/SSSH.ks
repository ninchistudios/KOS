core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
set terminal:width to 60.
set terminal:height to 40.
CLEARSCREEN.

print "########################".
print "# KERPERIUM BOOTLOADER #".
print "########################".
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
  copypath("0:/common/Utils.ks", "").
  copypath("0:/FalconHopper/FalconHopperI.ks", "").
  copypath("0:/fall/controllers/ascentController", "").
  copypath("0:/fall/models/hoverslamModel", "").
  // runoncepath("Utils").
  runoncepath("FalconHopperI").
  // ## END CONFIGURE MISSION ##
}

if ch = "A" { // ABORT
  print "# FLIGHT ABORTED #".
}
