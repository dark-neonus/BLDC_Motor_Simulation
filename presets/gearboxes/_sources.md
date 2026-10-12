# Gearbox presets — sources

All gearbox values are **estimates** for the class (`source: estimated`), not a specific product's datasheet.

| Preset | Ratio / η | Backlash | Contact stiffness | Basis |
|---|---|---|---|---|
| planetary-4, -6, -9 | single-stage planetary, η 0.90–0.92 | 0.3° (18 arcmin) | 1 N·m/arcmin | Small planetary reducers: 10–30 arcmin backlash and 1–10 N·m/arcmin torsional stiffness (mechanical.md EQ-MECH-04 typical values); gimbal-size units sit at the low end. |
| cycloidal-20, -50 | cycloidal, η 0.85 | 0.05° (3 arcmin) | 10 N·m/arcmin | Cycloidal drives are low-backlash and stiffer than planetary of the same size; upper end of the same range. |

Contact damping is not given; the build uses ζ = 0.1 of the reduced inertia (D-016).
