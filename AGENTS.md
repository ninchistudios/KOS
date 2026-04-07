# About

These are kOS scripts for a 2026 new KSP RSS RO RP-1 career mode playthrough.

# Principles

- kOS everything where possible (except for Science-Core sounding rockets)
- Launching from Woomera (~31deg S), not KSC.
- preserve exisiting comments and comment blocks in scripts for documentation and future standardisation
- all flight config variables go in the `CONFIGURE FLIGHT` block at the top of the script
- no hardcoded variables for telemetry even if they are not expected to change, use actual data
- preserve the nested structure of `doFlightTriggers()`
- avoid LEO junk. All jettisonable fairings deployed during ascent as soon as atmo drag is no longer a factor (unless required for re-entry). All boost stages should either splashdown ballistically or be capable of remote deorbiting where practicable. Junk with a periapsis above 600km shall be considered a planning failure.
