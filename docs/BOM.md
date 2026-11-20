# Lime Flyer: BOM and weight budget

Config: conventional micro plane. Foam wing with ailerons and a small centerline dihedral (about 2-3 deg per side, ~17 mm tip rise), printed or foam fuselage with a tail boom, flat stabilizer with elevator (250 span x 108 chord mm = 270 cm2, V_H 0.6, tail arm 400 mm from the wing quarter-chord to the tail quarter-chord, ~12-14 g), fixed fin (100 high x 96 chord mm = 96 cm2, rudder optional). Tractor or pusher motor, ~800 mm span, 2S. Estimated AUW is about 223 g: ~55 g of printed parts (owner-reported total, table below), a flat single-layer wing (~38 g est.), and electronics and fasteners. UNDER the 250 g limit (~27 g margin) on estimates, so weigh everything before flying. Target is under about 240 g.
Control: FS-i6, 3 channels plus throttle (aileron, elevator, throttle; rudder on a 4th if fitted). No flight controller and no mixing needed.

All weights and prices are ESTIMATES until measured. Verify before buying.

## Already owned
| Item | Notes |
|---|---|
| FS-i6 transmitter | AFHDS 2A |
| 3S LiPo charger | also charges 2S; needs a 2S balance lead |
| Bambu A1, PLA/PETG | motor mount, battery tray, fuselage parts |
| Nano AM32 ESCs (Foxair, 25 A) | oversized for a ~4 A motor, which is fine; ~4-6 g estimate, weigh one. Confirm 2S rating and whether it has a BEC |
| RS2205 2300KV motor (2 good, 1 bent shaft) | ~30 g. Flight motor, on 2S. Bench-test thrust and current first (expect roughly 8-25 A depending on prop) |
| 2S 800 mAh LiPo | flight pack (40-46 g measured). Check its C rating (the motor may pull 10+ A) |
| EP-6035 prop (6.0 x 3.5 in) | flight prop. Loads the motor hard at full throttle; test restrained with the watt meter and set a throttle limit in the radio |
| Receiver (FS2A) | 4 channel AFHDS 2A, PWM outputs, ~1-2 g (listings; weigh it). 4 channels covers aileron (Y-lead), elevator, throttle and an optional rudder |
| Micro servos | 2 aileron, 1 elevator, plus spares |
| Foam board | wing (flat for the first flight), stabilizer, fin, fuselage |
| Spare 5 V BEC | use if the ESC has no 5 V output (check the pinout with a meter) |

## Printed parts (measured)
Per-part rows are from the earlier print set and are now stale; the total is the latest reported figure. Update the rows when re-weighed.
| Part | Weight |
|---|---|
| Fuselage top half | 37.1 g |
| Fuselage bottom half | 6.5 g |
| Motor mount | 1.5 g |
| Main tail boom | 6.8 g |
| Tail (stabilizer and fin, printed) | 14.3 g |
| **Total printed (owner-reported, after changes)** | **~55 g** |

## To buy
| Item | Weight | Est. cost | Notes |
|---|---|---|---|
| Wire, Y-lead, connectors, glue, tape, hardware | ~10 g | ~$8 | whatever you do not already have |
| Wing spar | ~3-6 g | $0-5 | 3 mm board is only ~0.2x as stiff as 5 mm: add a spar (carbon rod, skewer or dowel) plus a tape strip (a flat wing has no KF layer to help) |
| **Total** | **~223 g** | **~$0-15** | ~27 g under 250 g on estimates only; weigh the finished plane before flying |

Optional: a 4th micro servo (~$4, ~4 g) for a rudder. 2-blade 5x3 props (~$5) if you want a lighter, more efficient baseline.

## Build notes
- Cut 2 wing sections that add up to 900 mm. The dihedral joint at the centerline costs about 50 mm per side (100 mm total), leaving the 800 mm span used everywhere else.
- First flight: flat 3 mm board wing, no KF step. If it stalls early or feels draggy, add a KF step layer later (about +12 g est.). If you do, leave the 50 mm at each joint end bare: no step there.

## Checks before the first flight
1. Bench test the motor on the EP-6035 with a scale and watt meter, restrained: confirm thrust is at least 1.5x the weight and note the current at full and half throttle. Set a throttle limit in the radio.
2. Balance the CG at the spot the plan gives (target 35% chord, 53 mm back from the wing leading edge; neutral point is estimated at ~68 mm, so static margin 10%; start a little forward, around 50 mm, and move aft after test glides); adjust with battery position.
3. Servo directions with the props off: stick right raises the right aileron and drops the left; stick back raises the elevator.
4. Range check and failsafe set on the FS-i6 (throttle cut, controls neutral).

## Canada
Under 250 g: no registration or pilot certificate needed. Check Transport Canada's current rules before flying.
