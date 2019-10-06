set center to ship:partstagged("center").
set boost to ship:partstagged("boost").
set innerboost to ship:partstagged("innerboost").
set launch to ship:partstagged("launch").

set gridfins to ship:partstagged("gridfin").

function activateCenterEngines { for e in center { e:activate(). } }
function shutdownCenterEngines { for e in center { e:shutdown(). } }

function activateBoostEngines { for e in boost { e:activate(). } }
function shutdownBoostEngines { for e in boost { e:shutdown(). } }

function activateInnerBoostEngines { for e in innerboost { e:activate(). } }
function shutdownInnerBoostEngines { for e in innerboost { e:shutdown(). } }

function activateLaunchEngines { for e in launch { e:activate(). } }
function shutdownLaunchEngines { for e in launch { e:shutdown(). } }

function shutdownBooster {
  shutdownCenterEngines().
  shutdownBoostEngines().
  shutdownInnerBoostEngines().
  shutdownLaunchEngines().
}

// function activateBurnedTexture {
//   local maintank to ship:partstagged("maintank")[0].
//   maintank:getModule("FStextureSwitch2"):doEvent("Previous Texture").
//
//   local octaweb to ship:partstagged("octaweb")[0].
//   octaweb:getModule("FStextureSwitch2"):doEvent("Next Paint").
// }

function limitGridFinAuthority {
  declare parameter limit.

  for fin in gridfins {
    fin:getmodule("ModuleControlSurface"):setfield("authority limiter", limit).
  }
}

function controlFromBooster {
  set guidance to ship:partstagged("boosterguidance")[0]:getModule("ModuleCommand").
  guidance:doEvent("Control From Here").
}
