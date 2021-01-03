@LAZYGLOBAL OFF.

//
// param AGL :
// param HAGL :
// param Tmin :
// returns
function hoverThrottle {
  parameter AGL,HAGL,Tmin.
  // thrott = TWRd x Fg / AVAILABLETHRUST
  // TWRd = (0.00012 * dV^3) + (0.000514286 * dV^2 + (0.003 * dV) +0.998286)
  // dV = (0.0732601 * dH^3) - (17.326 * dH)
  local dH is AGL - HAGL. // delta between desired height and actual height
  //print "dH:" + ROUND(dH,3) at (TERMINAL:WIDTH - 13,TERMINAL:HEIGHT - 13).
  local dV is desiredVv(dH). // desired surface velocity based on dH
  //print "dV:" + ROUND(dV,3) at (TERMINAL:WIDTH - 13,TERMINAL:HEIGHT - 14).
  local TWRd is desiredTWR(dV, ship:verticalspeed). // desired TWR
  //print "TWRd:" + ROUND(TWRd,3) at (TERMINAL:WIDTH - 15,TERMINAL:HEIGHT - 15).
  // local thrott is Fg * Tf / COS(vAngle) / AVAILABLETHRUST.
  local vthrott is TWRd * Fg / max(1,AVAILABLETHRUST). // throttle assuming vertical
  //print "vthrott:" + ROUND(vthrott,3) at (TERMINAL:WIDTH - 18,TERMINAL:HEIGHT - 16).
  local thrott is vthrott / COS(vAngle).
  //print "thrott:" + ROUND(thrott,3) at (TERMINAL:WIDTH - 17,TERMINAL:HEIGHT - 17).
  if (thrott < Tmin) {
    set thrott to Tmin.
    // TODO if we're still accelerating up we need to shutdown some engines
  } else if (thrott > 1) {
    set thrott to 1.
  }
  return thrott.
}

//
// param LAUNCH_AGL :
// param AGL_TWEAK :
// returns
function distanceToGround {
  parameter LAUNCH_AGL,AGL_TWEAK.
  return altitude - body:geopositionOf(ship:position):terrainHeight - LAUNCH_AGL - AGL_TWEAK.
}

//
function stoppingDistance {
  local grav is constant():g * (body:mass / body:radius^2).
  local maxDeceleration is (ship:availableThrust / ship:mass) - grav.
  return ship:verticalSpeed^2 / (2 * maxDeceleration).
}

//
// param AGL :
// returns
function predictedRadalt {
  parameter AGL.
  local s0 is AGL. // AGL of the nozzles
  local u is ship:verticalspeed. // the current velocity, -ve up
  local a is ship:sensors:acc:y. // this appears to be very unreliable
  // t is the time to v = 0 under the current a: v=u+at
  local t is (0 - u) / a.
  // distance to peak: s=ut+1/2at^2
  local s is (u * t) + (0.5 * a * (t^2)).
  set PREDICTED to s + s0. // TODO is this correct?
  return PREDICTED.
}


// basic function to calculate the desired vertical speed based on the distance from the target height
// param dH1 :
// returns
function desiredVv {
  parameter dH1.
  return max(-20,min(10,(-0.00000533333 * dH1^3) + (0.000000000000000000243714 * dH1^2) - (.0466667 * dH1))).
}

// basic function to calculate the desired TWR based on the desired vertical speed and current vertical speed
// param dv0 :
// param v0 :
// returns
function desiredTWR {
  parameter dv0,v0.
  local dv1 is dv0 - v0.
  // return max(0.8,min(1.3,(0.00012 * dV1^3) + (0.000514286 * dV1^2 + (0.003 * dV1) +0.998286))).
  return max(0,min(1.3,(0.038118 + (0.961882 * (constant:e ^ (0.0770805 * dv1)))))).
}
