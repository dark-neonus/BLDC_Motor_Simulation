---
title: References
---

# References

Every equation in the physics pages cites one of these sources by key, e.g. `[Krishnan2010]`. Identifiers were checked on 2026-10-09 (DOI resolves, ISBN matches the publisher's listing, or the datasheet URL is the manufacturer's).

## Books

| Key | Reference | Used for |
|---|---|---|
| `[Krishnan2010]` | R. Krishnan, *Permanent Magnet Synchronous and Brushless DC Motor Drives*. CRC Press, 2010. ISBN 978-0-8247-5384-9. | PMSM/BLDC models, dq transformation, six-step drive, sensorless control |
| `[Mohan2014]` | N. Mohan, *Advanced Electric Drives: Analysis, Control, and Modeling Using MATLAB/Simulink*. Wiley, 2014. ISBN 978-1-118-48548-4. | Space vectors, dq models, FOC, SVPWM |
| `[HendershotMiller2010]` | J. R. Hendershot, T. J. E. Miller, *Design of Brushless Permanent-Magnet Machines*. Motor Design Books, 2010. ISBN 978-0-9840687-0-8. | Windings, winding factor, cogging, losses, thermal behaviour, geometry |
| `[HolmesLipo2003]` | D. G. Holmes, T. A. Lipo, *Pulse Width Modulation for Power Converters: Principles and Practice*. Wiley-IEEE Press, 2003. ISBN 978-0-471-20814-3. | Carrier PWM, SVPWM / min-max injection, dead time, overmodulation |
| `[Hairer1993]` | E. Hairer, S. P. Nørsett, G. Wanner, *Solving Ordinary Differential Equations I: Nonstiff Problems*, 2nd ed. Springer, 1993. ISBN 978-3-540-56670-0. | Runge–Kutta methods, stability regions, step-size control |

## Papers

| Key | Reference | Used for |
|---|---|---|
| `[Lee2010]` | J. Lee, J. Hong, K. Nam, R. Ortega, L. Praly, A. Astolfi, "Sensorless control of surface-mount permanent-magnet synchronous motors based on a nonlinear observer," *IEEE Trans. Power Electronics*, vol. 25, no. 2, pp. 290–297, 2010. [doi:10.1109/TPEL.2009.2025276](https://doi.org/10.1109/TPEL.2009.2025276) | Nonlinear flux observer for sensorless FOC |
| `[Ortega2011]` | R. Ortega, L. Praly, A. Astolfi, J. Lee, K. Nam, "Estimation of rotor position and speed of permanent magnet synchronous motors with guaranteed stability," *IEEE Trans. Control Systems Technology*, 2011. [PDF (author copy)](https://cas.mines-paristech.fr/~praly/Telechargement/Journaux/2010_IEEE_CST-Ortega-Praly-Astolfi-Lee-Nam.pdf) | Observer stability proof, gain choice |
| `[Karnopp1985]` | D. Karnopp, "Computer simulation of stick-slip friction in mechanical dynamic systems," *J. Dynamic Systems, Measurement, and Control*, vol. 107, no. 1, pp. 100–103, 1985. [doi:10.1115/1.3140698](https://doi.org/10.1115/1.3140698) | Stick–slip friction model with a velocity dead band |
| `[Tremblay2009]` | O. Tremblay, L.-A. Dessaint, "Experimental validation of a battery dynamic model for EV applications," *World Electric Vehicle Journal*, vol. 3, 2009. [doi:10.3390/wevj3020289](https://doi.org/10.3390/wevj3020289) | Battery model parameterised from datasheet discharge curves |

## Application notes and project documentation

| Key | Reference | Used for |
|---|---|---|
| `[TI-BPRA073]` | Texas Instruments, "Field Orientated Control of 3-Phase AC-Motors," application report BPRA073, 1998. [PDF](https://www.tij.co.jp/jp/lit/an/bpra073/bpra073.pdf) | Clarke/Park conventions, FOC structure |
| `[ODrive]` | ODrive Robotics documentation. [https://docs.odriverobotics.com](https://docs.odriverobotics.com) | Practical gain design, Kt ≈ 8.27/Kv, calibration routines |
| `[SimpleFOC]` | SimpleFOC documentation. [https://docs.simplefoc.com](https://docs.simplefoc.com) | Open-loop/FOC modes, sensor handling on hobby hardware |
| `[VESC]` | VESC project. [https://vesc-project.com](https://vesc-project.com) | Flux-observer usage, motor detection (R, L, λ measurement) |

## Sensor datasheets

| Key | Reference | Used for |
|---|---|---|
| `[AS5047P]` | ams OSRAM (Infineon), *AS5047P 14-bit on-axis magnetic rotary position sensor*, datasheet. [PDF](https://www.infineon.com/row/public/documents/24/49/infineon-as5047p-datasheet-en.pdf) | Resolution, latency (DAEC), max speed |
| `[AS5600]` | ams OSRAM (Infineon), *AS5600 12-bit programmable contactless potentiometer*, datasheet. [PDF](https://www.infineon.com/assets/row/public/documents/24/49/infineon-as5600-datasheet-en.pdf) | Resolution, output modes, recommended air gap |
| `[MT6701]` | MagnTek, *MT6701 magnetic angle sensor*, datasheet Rev. 1.0. [PDF](http://www.magntek.com.cn/upload/MT6701_Rev.1.0.pdf) | 14-bit core, ~5 µs system delay, ±1° INL |

## How to cite in the docs and code

- Docs: `… per [Krishnan2010, §4.2].`
- Rust/Python: `/// EQ-MOT-04 [Krishnan2010 §4.2]` in the doc comment of the function implementing the equation.
