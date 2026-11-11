# Citrus Tiltrotor — BOM and weight budget

Config: Y tilt-tri (2 front tilting motors + 1 fixed rear lift motor), conventional wing + V-tail.
Target: ~1300 mm span, ~200 mm chord, ~850 g AUW, **3S** (owned charger is 3S-only).
Wing: foam board + carbon rod spar. Fuselage, tail, nacelles, mounts: printed PLA/PETG.

All weights/prices are ESTIMATES until measured. Verify before buying.

## Already owned
| Item | Notes |
|---|---|
| 3x RS2205 2300KV | ~30 g each (weigh them) |
| 3x 25A ESC, 2-4S | treat as 25 A until measured |
| FS-i6 transmitter | 6 ch stock, AFHDS 2A |
| 3S LiPo charger | max 3S |
| 2S 800 mAh LiPo | bench testing only (no props) |
| Bambu A1, PLA/PETG | 256^3 mm bed |
| navX2 | not used (not supported by ArduPilot) |

## To buy
| Item | Est. cost | Notes |
|---|---|---|
| Flight controller (ArduPilot) | $40-50 | Needs **9 PWM outputs** (3 motors + 2 tilt + 2 aileron + 2 V-tail). Verify output count and ArduPilot support. SpeedyBee F405 WING APP (full size) likely; Matek F405-WING V2 may have too few outputs. |
| 3S 2200 mAh LiPo, 50C+ | $15-20 | ~190 g |
| 2x metal-gear servos (tilt) | $10-15 | |
| 4x 9 g servos (2 aileron, 2 V-tail) | $10-15 | |
| FS-iA6B receiver (if not owned) | $12 | iBUS |
| 5 V BEC (3 A) | $5-8 | servo power, if FC lacks enough |
| 5" props (5045 / 5030), CW + CCW, spares; optionally 6" for testing | $5-12 | |
| Foam board (5 mm), 3-4 mm carbon rod, boom tube | $10-15 | |
| Wire, XT60, balance lead, heat shrink | ~$10 | |
| Inline watt meter | ~$10 | bench thrust/current test |
| GPS + compass (optional) | $15-20 | RTL; not needed for first hover |

Estimated spend: ~$100-150 depending on what you skip.

## Weight budget (estimate)
| Part | g |
|---|---|
| 3x RS2205 | 90 |
| 3x ESC | 30 |
| FC | 10 |
| Receiver | 5 |
| 3S 2200 mAh | 190 |
| 2x tilt servos | 26 |
| 4x control servos | 36 |
| 3x props | 12 |
| Wiring, connectors, BEC | 40 |
| Carbon spar rods + boom | 40 |
| Foam wing | 130 |
| Fuselage, tail, nacelles, mounts | 240 |
| **Total** | **~850** (+10 with GPS) |

## 3S go/no-go (bench test)
Mount one motor + prop on a scale, watt meter inline, 3S pack.
- >= 0.8 kg thrust per motor: T/W ~2.8 at 850 g, workable.
- 0.7 kg: T/W ~2.5, marginal; try 6" props or cut weight.
- < 0.65 kg: 3S not viable; need 4S and a 4S charger.
