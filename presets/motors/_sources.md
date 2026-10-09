# Motor preset sources (P04.T09)

The presets are **generic size classes**, not specific products. Each value carries
`source: datasheet` (taken from or directly converted from a public listing) or
`source: estimated` (rules EQ-EST-01…03 in `docs/docs/physics/estimation.md`, or interpolated
between neighbouring sizes). Retail listings were collected on 2026-10-09; they are secondary
sources and sometimes disagree. Where they disagree, the range is given and a middle value used.

**Conventions.** Hobby datasheets give resistance and inductance **line-to-line**; the presets
store star-equivalent phase values (half). Hobby Kv is measured at no load against the DC
supply and is used as entered (LL peak, EQ-CONV-11).

| Class | Public data found (ranges) | Picked | Gaps → estimated |
|---|---|---|---|
| 2804 | iFlight GBM2804H-100T: 12N14P, 10 Ω, 39 g, 1468–1622 rpm @ 10 V (≈150 rpm/V), ≤25 W. iFlight GM2804: 9 Ω, 1602–1771 rpm @ 10 V; RobotShop GM2804 listing: 5.57 Ω, "100 KV". | 150 rpm/V, 9 Ω LL, 39 g | L, J, thermal, currents |
| 4108 | iFlight GM4108H-120T: 24N22P, 11.1 Ω, 124–125 g, 513–567 rpm @ 20 V (≈27 rpm/V), Φ47 × 32 mm, 1.5 A load current, "load torque 1200–1800 g·cm". Reseller copies only. | 27 rpm/V, 11.1 Ω, 124 g | L, J, thermal, peak current |
| 5010 | MAD 5010 EEE/IPE: 14 pole pairs, 200 KV 106 mΩ / 240 KV 94.6 mΩ / 310 KV 51 mΩ 46.8 A / 370 KV 38 mΩ, 162–181 g. Rctimer 5010: 12N14P, 360 KV 92 g, 260 KV 0.6 Ω 100 g. | 24N28P, 310 rpm/V, 51 mΩ, 162 g, 46.8 A | L, J, thermal |
| 6010 | DJI E2000 6010: 130 rpm/V, 230 mΩ LL, 230 g. Vector Technics 6010 PRO: 160 rpm/V, 240 g. (Gimbal-wound GB6010: 15.5 rpm/V, 10.6 Ω, 242 g; XTG6010 11.8 Ω, 232 g.) | 24N28P (estimated), 130 rpm/V, 230 mΩ, 230 g | slots/poles, L, J, thermal, currents |
| 6020 | DJI RoboMaster GM6020 (reseller table): 10 pole pairs, 1.8 Ω phase, 5.78 mH, Kv 13.33 rpm/V, Kt 741 mN·m/A, 1.2 N·m / 1.62 A rated, 468 g, 125 °C max winding. Only one manufacturer found. | as listed; 24 slots estimated | slots, J, thermal, peak current |
| 8010 | No 8010 datasheet found. T-Motor MN8012 KV100: 36N42P, 85 mΩ, 351 g. MN8014 KV100: 65 mΩ, 52 A (180 s). MAD8108 100 KV: 186 mΩ, 250–265 g. | interpolated: 100 rpm/V, 100 mΩ, 300 g, 40 A | all (interpolated) |
| 8108 | Hobbywing M8108 Light: 36N42P, 100 KV 165.2 mΩ 238 g (85 KV 239.5 mΩ, 150 KV 82.8 mΩ). Hobbywing M8108 HP 100 KV 163.3 mΩ. MAD8108 100 KV: 186 mΩ, 250 g, 24 A (60 s). | 100 rpm/V, 175 mΩ, 245 g, 24 A | L, J, thermal |
| 10015 | CubeMars AK10-9 V2 (motor stage of a 9:1 actuator): 21 pole pairs, 100 rpm/V, 90 mΩ / 331 µH LL, 18.5 A rated / 50 A peak, 38 N·m peak at the output. T-Motor MN1010 KV90: 46 mΩ, 630 g, 56 A (180 s). | 36N42P, 100 rpm/V, 90 mΩ, 331 µH, 630 g, 50 A | J, thermal |
| 12020 | No 120 mm class datasheet found. Scaled from CubeMars RO100 KV55 (Φ108 mm): 21 pole pairs, 143 mΩ / 137 µH LL, 62 A / 12 N·m peak, 525–710 g. | estimated: 50 rpm/V, 120 mΩ, 120 µH LL, 1.1 kg, 70 A | all (scaled) |

Inrunner variants: none of these classes is commonly sold as an inrunner for gimbal/robot use
(the CubeMars RI100, Φ104 × 26 mm, 14 pole pairs, 105 rpm/V, 126 mΩ / 366.7 µH LL, is the
nearest), so no inrunner presets were added.

## Cogging (EQ-MOT-08)

No datasheet in the table lists cogging torque. Each preset carries one harmonic with
amplitude **2 % of Kt·I_peak** (`source: estimated`), the middle of the 0.5–5 % of rated
torque typical for slotted gimbal motors `[HendershotMiller2010]`. Phase 0; N_c = LCM(slots, 2p).

## Peak-torque check (Kt · I_peak vs datasheet, ±30 %)

| Class | Kt (derived) | Kt · I | Datasheet torque | Difference |
|---|---|---|---|---|
| 10015 | 82.7 mN·m/A | 4.13 N·m at 50 A | 38 N·m / 9 = 4.22 N·m | −2 % ✓ |
| 6020 | 620 mN·m/A | 1.00 N·m at 1.62 A (rated) | 1.2 N·m rated | −16 % ✓ |
| RO100 (12020 basis) | 150 mN·m/A | 9.3 N·m at 62 A | 12 N·m | −22 % ✓ |
| 4108 | 306 mN·m/A | 0.46 N·m at 1.5 A | "load torque" 0.12–0.18 N·m | not comparable (test-load figure, not a peak rating) |

The other classes list no torque (drone motors are rated by thrust).

## Sources

- iFlight GBM2804H-100T: <https://shop.iflight.com/ipower-gimbal-brushless-motor-gbm2804h-100t-pro218>
- iFlight GM2804: <https://shop.iflight.com/ipower-gm2804-gimbal-motor-pro1153>; RobotShop listing <https://www.robotshop.com/products/ipower-gm2804-gimbal-motor-w-as5048a-encoder>
- iFlight GM4108H-120T: <https://shop.iflight.com/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217>; <https://www.robotshop.com/products/ipower-gbm4108h-120t-gimbal-motor>
- MAD 5010 IPE V3 / EEE: <https://mad-motor.com/products/mad-components-5010-ipe-v3>; <https://druav.com/en-us/products/mad-5010-eee-drone-motor>
- Rctimer 5010: <https://rctimer.com/rctimer-5010-260kv-multirotor-brushless-motor40mm-shaft-p0132.html>
- DJI 6010 (E2000): <https://www.bhphotovideo.com/c/product/1223963-REG/dji_cp_ep_000088_standard_6010_motor_for.html>
- Vector Technics 6010 PRO 160 KV: <https://www.vectortechnics.com/motors-products/6010-pro-160-kv>
- SteadyWin GB6010: <https://aifitlab.com/products/steadywin-gb6010-motor>
- DJI RoboMaster GM6020: <https://aifitlab.com/products/dji-robomaster-gm6020-brushless-dc-motor>
- T-Motor MN8012 KV100: <https://store.uniteduav.com/products/mn8012-kv100>; MN8014 KV100: <https://ca.robotshop.com/products/tmotor-uav-brushless-motor-antigravity-mn8014-kv100>
- MAD8108 100 KV: <http://madcomponents.co/index.php/mad8108-100kv/>
- Hobbywing M8108: <https://hobbywing.com/en/products/hm8108>
- CubeMars AK10-9 V2: <https://www.cubemars.com/goods.php?id=982>; <https://www.robotshop.com/products/cubemars-cubemars-ak10-9-v20-kv100-brushless-dc-motor-robot>
- T-Motor MN1010 KV90: <https://www.robotshop.com/products/tmotor-uav-brushless-motor-mn1010-kv90>
- CubeMars RO100 KV55: <https://www.cubemars.com/goods-1159-RO100.html>; <https://openelab.io/products/cubemars-ro100-kv55-hall>
- CubeMars RI100 KV105: <https://openelab.io/products/cubemars-ri100-frameless-inrunner-series>
