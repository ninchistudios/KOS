core:part:getmodule("kOSProcessor"):doevent("Open Terminal").

CLEARSCREEN.
print "##################################".
print "# KERPERIUM BOOTLOADER boot01.ks #".
print "##################################".
print " ".

set misFile to "0:/PolarContract02.ks".
print "Mission Boot File: " + misFile.
print "Go Flight: [G]".
print "Abort: [A]".
set validInput to false.

until validInput {
  set ch to terminal:input:getchar().
  if (ch = "G" or ch = "A") set validInput to true.
}

if ch = "G" { // GO FLIGHT
  print "# GO FLIGHT #".
  runpath(misFile).
}

if ch = "A" { // ABORT
  print "# FLIGHT ABORTED #".
}
