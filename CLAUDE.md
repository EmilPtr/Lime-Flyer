# Lime Flyer

Sub-250 g RC plane, hand-launched. Owner builds it at home (Bambu A1 printer, hot glue, foam board). Based in Canada: under 250 g needs no registration; check Transport Canada's current rules.

## Decisions (do not re-open without asking)
- Conventional layout (ailerons, elevator, fixed fin, rudder optional). NOT a flying wing.
- Tractor: motor and prop on the nose.
- Wing: 5 mm foam board, KF step, ~800 x 150 mm. No carbon rod required. Small dihedral, about 2-3 deg per side (~17 mm tip rise), as a single break at the centerline.
- Power: RS2205 2300KV on 2S, EP-6035 prop (6.0 x 3.5 in), nano AM32 25 A ESC with BEC, 2S 800 mAh LiPo. Owner plans to stay at or under about 50% throttle and set a throttle limit.
- Radio: FS-i6 with an AFHDS 2A receiver. No flight controller, no mixing.
- The earlier tilt-tri VTOL was dropped; its files are in git history (commit 1ddb056, under archive/tilt-tri/).

## Files
- `docs/BOM.md`: what is owned, what is left to buy, weight budget. All weights and prices are estimates until measured.
- `docs/design-guide.html`: single-file interactive guide (lift, drag, thrust, CG, nose layout) with a CG worksheet. Defaults mirror the BOM; keep them in sync when parts change.

## Working notes
- Thrust, current and CG numbers are placeholders until the owner bench-tests the motor and weighs parts. Update the BOM and guide defaults with measured values when given.
- No browser is available in the dev sandbox, so verify guide changes with `node --check` on the extracted script and a stubbed run.
- Ask before pushing. Remote is `origin` = github.com/EmilPtr/Lime-Flyer.
