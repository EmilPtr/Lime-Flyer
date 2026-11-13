# Lime Flyer: BOM and weight budget

Config: conventional micro plane. Foam wing with ailerons and a small centerline dihedral (about 2-3 deg per side, ~17 mm tip rise), printed or foam fuselage with a tail boom, flat stabilizer with elevator, fixed fin (rudder optional). Tractor or pusher motor, ~800 mm span, 2S. Target AUW 170-220 g (about 190 g typical, with the 30 g RS2205), well under the 250 g limit.
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
| Receiver | confirm it is AFHDS 2A (e.g. FS-iA6B) with PWM outputs; old AFHDS receivers do not bind to the FS-i6 |
| Micro servos | 2 aileron, 1 elevator, plus spares |
| Foam board | wing (KF step), stabilizer, fin, fuselage |
| Spare 5 V BEC | use if the ESC has no 5 V output (check the pinout with a meter) |

## To buy
| Item | Weight | Est. cost | Notes |
|---|---|---|---|
| Wire, Y-lead, connectors, glue, tape, hardware | ~10 g | ~$8 | whatever you do not already have |
| Wing spar (optional) | ~0-5 g | $0-5 | carbon rod not required; try the foam board alone, add a skewer, dowel or tape stiffener if the wing flexes |
| **Total** | **~170-220 g** | **~$0-15** | Well under 250 g |

Optional: a 4th micro servo (~$4, ~4 g) for a rudder. 2-blade 5x3 props (~$5) if you want a lighter, more efficient baseline.

## Checks before the first flight
1. Bench test the motor on the EP-6035 with a scale and watt meter, restrained: confirm thrust is at least 1.5x the weight and note the current at full and half throttle. Set a throttle limit in the radio.
2. Balance the CG at the spot the plan gives; adjust with battery position.
3. Servo directions with the props off: stick right raises the right aileron and drops the left; stick back raises the elevator.
4. Range check and failsafe set on the FS-i6 (throttle cut, controls neutral).

## Canada
Under 250 g: no registration or pilot certificate needed. Check Transport Canada's current rules before flying.
