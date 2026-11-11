# Citrus Tiltrotor — BOM and weight budget

Config: Y tilt-tri (2 front tilting motors + 1 fixed rear lift motor), conventional wing + V-tail.
Target: ~1300 mm span, ~210 mm chord, ~1030 g AUW, 4S.

All weights/prices are ESTIMATES until measured. Verify before buying.

## Already owned
| Item | Notes |
|---|---|
| 3x RS2205 2300KV | ~30 g each (weigh them) |
| 3x 25A ESC, 2-4S | treat as 25 A until measured |
| FS-i6 transmitter | 6 ch stock, AFHDS 2A |
| 3S LiPo charger | cannot charge 4S |
| 2S 800 mAh LiPo | bench testing only (no props) |
| Bambu A1, PLA/PETG | 256^3 mm bed |
| navX2 | not used (not supported by ArduPilot) |

## To buy
| Item | Est. cost | Notes |
|---|---|---|
| Flight controller (ArduPilot) | $40-50 | Needs **9 PWM outputs** (3 motors + 2 tilt + 2 aileron + 2 V-tail). Verify output count and ArduPilot support. SpeedyBee F405 WING APP (full size) likely; Matek F405-WING V2 may have too few outputs. |
| 4S 1500 mAh LiPo, 75C+ | $20 | |
| 4S balance charger | $25-35 | Skip for now if running 3S bench tests |
| 2x metal-gear servos (tilt) | $10-15 | |
| 4x 9 g servos (2 aileron, 2 V-tail) | $10-15 | |
| FS-iA6B receiver (if not owned) | $12 | iBUS |
| 5 V BEC (3 A) | $5-8 | servo power, if FC lacks enough |
| 5" props (5030/5x3 and 5045), CW + CCW, spares | $5-8 | |
| Carbon tube 10 mm (spar) + boom tube | $10-15 | |
| Wire, XT60, balance lead, heat shrink | ~$10 | |
| Inline watt meter | ~$10 | bench thrust/current test |
| GPS + compass (optional) | $15-20 | RTL; not needed for first hover |

Estimated spend: ~$120-180 depending on what you skip.

## Weight budget (estimate)
| Part | g |
|---|---|
| 3x RS2205 | 90 |
| 3x ESC | 30 |
| FC | 10 |
| Receiver | 5 |
| GPS (optional) | 10 |
| 4S 1500 mAh | 170 |
| 2x tilt servos | 26 |
| 4x control servos | 36 |
| 3x props | 12 |
| Wiring, connectors, BEC | 40 |
| Carbon spar + boom | 60 |
| Wing (6 PLA sections) | 300 |
| Fuselage, tail, nacelles, mounts | 240 |
| **Total** | **~1030** |
