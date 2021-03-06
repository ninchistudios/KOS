core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
set terminal:width to 60.
set terminal:height to 40.
CLEARSCREEN.

print "########################".
print "# KERPERIUM BOOTLOADER #".
print "#      HUEY CLASS      #".
print "########################".
print " ".
print "Abort: [A]".
print "Go Flight: [F]".

set validInput to false.
until validInput {
  set ch to terminal:input:getchar().
  if (ch = "F" or ch = "A") set validInput to true.
}

if ch = "F" { // GO FLIGHT

  // ## CONFIGURE MISSION ##
  print "# GO FLIGHT - CONFIRM MISSION CONFIG #".
  print "Abort: [A]".
  print "HUEY-22LE-003: [B]".
  print "HUEY-22LE-004: [C]".

  copypath("0:/common/Utils.ks", "").

  set validInput to false.
  until validInput {
    set ch to terminal:input:getchar().
    if (ch = "A" or ch = "B" or ch = "C") set validInput to true.
  }

  if ch = "B" {
    copypath("0:/HueyClass/Huey-22LE-003.ks", "").
    runoncepath("Huey-22LE-003").
  }

  if ch = "C" {
    copypath("0:/HueyClass/Huey-22LE-004.ks", "").
    runoncepath("Huey-22LE-004").
  }

  // ## END CONFIGURE MISSION ##
}

if ch = "A" { // ABORT
  print "# FLIGHT ABORTED #".
}
