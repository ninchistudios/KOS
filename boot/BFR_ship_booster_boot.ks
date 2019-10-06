copypath("0:/functions.ks", "").
copypath("0:/bfr/ascent.ks", "").
copypath("0:/bfr/bfr_util.ks", "").
copypath("0:/bfr/landing.ks", "").
copypath("0:/bfr/ship_booster.ks", "").

core:part:getmodule("kOSProcessor"):doEvent("Open Terminal").
set terminal:width to 45.
set terminal:height to 20.

run ship_booster.
