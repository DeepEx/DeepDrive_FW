# deepDrive MK1 — current and voltage calibration

The ES.046 rev. 2.0 power board senses each low-side shunt (0.5 mΩ, R16–R18) from
a point between the two footprint positions of a dual-shunt layout. With a single
shunt fitted, pads, solder and copper add to the sensed resistance, so the
nominal `CURRENT_SHUNT_RES` overstates current. Measured on a spare board
(2026-11-03, 5 A, 4-wire at the SP/SN pins): 0.718 / 0.627 / 0.651 mΩ, against
0.498 mΩ across the shunt body. The error differs per board and per phase.

The firmware keeps the nominal constants and applies a per-board calibration
stored in the hardware EEPROM area (`EEPROM_VARS_HW` 0–6). With nothing stored,
behaviour is identical to the nominal build.

## Terminal commands (VESC Tool terminal)

| Command | Purpose |
| --- | --- |
| `dd_cal` | Print calibration, effective shunt per phase, nominal and calibrated Vin |
| `dd_cal_shunt r1 r2 r3` | Set effective shunt per phase in mΩ (4-wire measurement at the SP/SN pins); gain = 0.5 / r |
| `dd_cal_gain g1 g2 g3` | Set per-phase current gains directly |
| `dd_cal_vin gain offset` | Input voltage: V = gain · V_nominal + offset |
| `dd_cal_vph factor` | Phase-voltage sense factor |
| `dd_cal_balance [s]` | With the motor running steadily (> 3 A, duty < 0.5), measure per-phase RMS and suggest gains that remove phase mismatch while keeping the average gain; stores nothing |
| `dd_cal_reset` | Restore nominal values and invalidate the stored calibration |

Setting commands are refused while the motor is running. Gains are limited to
0.5–1.5, voltage gains to 0.8–1.2 and the voltage offset to ±5 V. The magic word
is written last, so an interrupted store is never loaded.

## Procedure

1. Board out of the vehicle: inject 5–10 A with a calibrated supply in constant
   current from the low-side source pad to the 0 V bar, measure mV at the SP/SN
   pins, reverse polarity and average; `dd_cal_shunt r1 r2 r3`.
2. Board in the vehicle (no disassembly): `dd_cal_shunt` with the spare-board
   average as first approximation, then `dd_cal_balance` at steady load and
   apply the suggested `dd_cal_gain`.
3. `dd_cal_vin`: compare `dd_cal` Vin with a calibrated supply at two voltages,
   gain = ΔV_true / ΔV_reported, offset from one point.
4. After any change: FOC offset calibration, then motor detection (R, L, λ).
   R and L detected before calibration are wrong by the same factor.

The unbalance check (`m_curr_unbalance`) now uses the calibrated per-phase
readings, so a gain mismatch is no longer reported as current unbalance.
