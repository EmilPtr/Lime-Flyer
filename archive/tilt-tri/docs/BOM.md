# Citrus Tiltrotor — BOM and weight budget

Config: Y tilt-tri (2 front tilting motors + 1 fixed rear lift motor), straight foam wing, conventional tail (flat stabilizer with elevator, fixed fin).
Target: 1300 mm span, 200 mm chord, ~835 g AUW (`docs/geometry.md`), **3S** (owned charger is 3S-only).
Structure: 5 mm foam-board wing on a carbon centre tube (10x8, 600 mm) with 4 mm carbon rods outboard; boom is a second 10x8 tube. Fuselage, tail mounts, nacelles: printed PLA/PETG. See `docs/geometry.md` section 0 for why the tube is there.

All weights/prices are ESTIMATES until measured. Verify before buying.

## Already owned
| Item | Notes |
|---|---|
| 3x RS2205 2300KV | ~30 g each (weigh them) |
| 3x 25A ESC, 2-4S | treat as 25 A until measured; used as 3 separate ESCs (no 4-in-1 needed) |
| FS-i6 transmitter | 6 ch stock, AFHDS 2A |
| Old AFHDS (not 2A) 6ch receiver, 4ch receivers | **not usable**: AFHDS does not bind to the FS-i6, and 4 channels is too few |
| 3S LiPo charger | max 3S |
| 2S 800 mAh LiPo | bench testing only (no props) |
| Bambu A1, PLA/PETG | 256^3 mm bed |
| navX2 | not used (not supported by ArduPilot) |

## To buy
| Item | Est. cost | Notes |
|---|---|---|
| Flight controller: FlyingRC F4WSE Pro F405 | $40-50 | 12 PWM outputs (need 8). Target `FlyingRC_F4WSE_Pro` is only on FlyingRC's fork branch `add_FlyingRC_F4WSE_Pro_target`, **not in official ArduPilot**. 1 MB flash: confirm `Q_ENABLE` / QuadPlane is in the firmware. See "FC checks" below. 9.1 g, 5 V/5 A servo BEC, 5 V/3 A logic BEC |
| 3S 1800 mAh LiPo, 30C+ (Amazon.ca / Prime) | $15-22 | ~155 g and ~100 mm assumed; weigh it, check length <= ~105 mm, and put the real numbers in `BATT` in `tools/geometry.py`. XT60. 2200 mAh (190 g) is the fallback if you want more hover time |
| 2x MG90S metal-gear servos (tilt) | $5-10 | 13.4 g, ~2.2 kg/cm [A] vs ~1.6 kg/cm needed: tight. Measure travel (need 110 deg) and backlash. MG92B is the upgrade if it struggles |
| 3x SG90 servos (2 aileron, 1 elevator) | $5-8 | 9 g. Buy spares. Optional: MG90S on the elevator |
| FS-iA6B receiver | $12 | AFHDS 2A + iBUS to a free FC UART. Check the listing says AFHDS 2A and iBUS |
| 5" props (5045 / 5030), CW + CCW, spares; optionally 6" for testing | $5-12 | |
| Foam board 5 mm (700 x 500 sheets), 4 mm carbon rod | $10-15 | At least 2 sheets (wing halves use 700 x 400 of one). Rerun the nesting check with `SHEET = (700, 500)` |
| 10x8 carbon tube, 600 mm (wing) + 10x8, 570 mm (boom) | $15-25 | Aluminium 10x8 is a heavier, ~70 % as stiff substitute for the wing tube |
| Wire, XT60, balance lead, heat shrink | ~$10 | |
| Inline watt meter | ~$10 | bench thrust/current test |
| GPS + compass (optional) | $15-20 | RTL; not needed for first hover |

Removed from the list: separate 5 V BEC (the FC has a 5 V/5 A servo rail; add one back only if you want a spare), the second pair of V-tail servos (conventional tail now).

Estimated spend: ~$110-160 depending on what you skip.

## FC checks (do on the day it arrives, inside the return window)
1. USB power only, props off. Flash the firmware the seller names, confirm Mission Planner connects.
2. Confirm `Q_ENABLE` exists and can be set to 1 (1 MB flash may have QuadPlane trimmed). If not, try ArduPilot's custom build server, or return the board.
3. Confirm IMU, barometer and voltage readings; find out which IMU (ICM42605 or BMI270) your revision has.
4. Receiver on a free UART set to RC input (iBUS); confirm the channels move.
5. Output plan from the hwdef timer groups (outputs 1-2, 3-4, 5-8 share timers; 9-11 are no-DMA, PWM only):
   - motors on outputs 1-3 (output 4 unused)
   - servos on 5-8: tilt L, tilt R, aileron L, aileron R
   - elevator on 9
   - a later rudder would go on 10
6. Bench-test all outputs (3 motors + 5 servos), failsafe, and servo directions with props off.

## Weight budget (estimate; matches `docs/geometry.md`)
| Part | g |
|---|---|
| 3x RS2205 | 90 |
| 3x ESC | 30 |
| FC | 9 |
| Receiver | 5 |
| 3S 1800 mAh | 155 |
| 2x tilt servos (MG90S) | 27 |
| 3x control servos (SG90) | 27 |
| 3x props | 12 |
| Wiring, connectors | 40 |
| Carbon: centre tube 26 + rods 18 + boom 25 | 69 |
| Foam wing | 130 |
| Fuselage, tail, nacelles, mounts | 240 |
| **Total** | **~835** (+10 with GPS) |

Weigh the real battery, motors and servos and set them in `tools/geometry.py`, then rerun it to re-solve the CG.

## 3S go/no-go (bench test)
Mount one motor + prop on a scale, watt meter inline, 3S pack, restrained, safety glasses.
- >= 0.8 kg thrust per motor: T/W ~2.9 at 835 g, workable.
- 0.7 kg: T/W ~2.5, marginal; try 6" props or cut weight.
- < 0.65 kg: T/W < 2.3, 3S not viable; need 4S and a 4S charger.
This also checks the 680 g per motor assumed in the geometry script.

## Canada: before the first outdoor flight
Over 250 g: register the aircraft with Transport Canada and hold the Basic Operations certificate (online exam). Check Transport Canada's current rules. Indoors: bench and restrained tests only, no free hover in a room.
