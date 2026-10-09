# Supply preset sources (P04.T10)

## Open-circuit voltage curves (per cell, at rest)

Two shared tables are used. Published charts differ by a few tens of mV; the presets
use values inside the published ranges.

| SoC | NMC / LiPo (V) | LiFePO4 (V) |
|---|---|---|
| 1.0 | 4.20 | 3.40 |
| 0.9 | 4.08 | 3.35 |
| 0.8 | 3.98 | 3.32 |
| 0.7 | 3.90 | 3.30 |
| 0.6 | 3.82 | 3.27 |
| 0.5 | 3.74 | 3.26 |
| 0.4 | 3.68 | 3.25 |
| 0.3 | 3.62 | 3.22 |
| 0.2 | 3.55 | 3.20 |
| 0.1 | 3.45 | 3.00 |
| 0.05 | 3.30 | 2.90 |
| 0.0 | 3.00 | 2.50 |

- **LiFePO4:** the resting table of Wevolver's LiFePO4 chart (3.40 V full at rest … 2.50 V at the 0 % cutoff): <https://www.wevolver.com/article/lifepo4-voltage-chart-soc-voltages-for-32v12v24v48v-systems>; it agrees with <https://www.vatrerpower.com/blogs/news/lifepo4-voltage-chart-a-comprehensive-guide>.
- **NMC Li-ion:** 18650 charts give 4.2 V = 100 % … 3.3 V ≈ 10 % in 0.1 V steps (<https://cmbatteries.com/understanding-18650-battery-voltage-from-basic-to-advanced>); measured curves are not linear: they drop steeply below ~10 % and are flatter in the middle (<https://endless-sphere.com/sphere/threads/li-ion-battery-voltage-vs-state-of-charge.82345>, <https://mediatum.ub.tum.de/doc/1161059/1161059.pdf>). The preset follows that shape (3.74 V at 50 %); the 0 % point is a 3.0 V cutoff choice (sources give 2.5–3.0 V).
- **LiPo:** LiPo cells are LCO/NMC chemistry with the same 4.2 V full voltage (<https://uwarg-docs.atlassian.net/wiki/spaces/EL/pages/2177794083>); the NMC table is used as a proxy (**estimated**).

## Resistances and capacities

Cell capacity, R0 and the RC pair (R1, C1) are **estimated** class values: hobby LiPo packs (2.2–5 Ah,
4–8 mΩ per cell), 3.5 Ah NMC 18650 cells (~30 mΩ), LiFePO4 prismatic cells (4–10 mΩ). The RC
time constants (R1·C1 ≈ 8–80 s) are in the usual range for first-order Thevenin models.
The PSU output resistance (20 mΩ) is an estimate for the leads and output stage.
