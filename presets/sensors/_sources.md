# Sensor preset sources (P04.T10)

| Preset | Data used | Source |
|---|---|---|
| AS5600 | 12-bit, analog/PWM/I²C output | ams-OSRAM / Infineon AS5600 datasheet: <https://www.infineon.com/assets/row/public/documents/24/49/infineon-as5600-datasheet-en.pdf> |
| AS5047P | 14-bit SPI, ABI up to 1024 ppr, DAEC (near-zero latency), up to 28 krpm | ams-OSRAM AS5047P product page: <https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5047p-> |
| MT6701 | 14-bit SSI/I²C, ABZ 1–1024 ppr, 12-bit PWM/analog, < 5 µs system delay, 55 krpm, ±1° typ. linearity | MagnTek MT6701 datasheet (LCSC copy): <https://atta.szlcsc.com/upload/public/pdf/source/20240409/995E18415A669787D9A549675AADF1FD.pdf>; driver notes: <https://components.espressif.com/components/espp/mt6701/versions/1.3.3/readme> |

INL and noise follow the chip table in `docs/docs/physics/sensors.md` (datasheets `[AS5047P]`,
`[AS5600]`, `[MT6701]`): AS5047P ±0.8°, 0.068° 1σ; AS5600 ±1°, 0.015° 1σ (SF = 00); MT6701 ±1°,
noise not specified (0.05°, **estimated**). Earlier presets had an estimated AS5047P INL of 0.34°;
P08.T03 aligned them with the spec.

**Estimated** (marked `source: estimated`): the MT6701 noise, and all `update_rate`/`latency`
values for polled interfaces, which depend on the microcontroller, not the chip.
The Hall set, the 1000-CPR optical encoder and the shunt ADC are generic parts with no single source.
