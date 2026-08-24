# Worst-case battery / forced-awake sweep

Each case is generated from the **SPICE netlist exported by the actual KiCad hierarchy**.
The ESP radio burst (20 -> 180 mA) is deliberately aligned with forced-awake startup.
For this stress regression only, the 3V3 input proxy is converted to a conservative
constant-power approximation at 88% efficiency, and the 5V boost gets additional
low-voltage input loading so its nominal 56-ohm model does not become optimistic.

| OCV | Rint | VBAT terminal | SYS | 3V3 | +5V | UNNI_AC | Ibat | Rails |
|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 4.2 V | 0.1 Ω | 4.175 V | 4.164 V | 3.286 V | 4.995 V | 4.737 V | 249 mA | PASS |
| 4.2 V | 0.3 Ω | 4.125 V | 4.114 V | 3.286 V | 4.995 V | 4.737 V | 249 mA | PASS |
| 4.2 V | 0.5 Ω | 4.074 V | 4.063 V | 3.286 V | 4.995 V | 4.737 V | 252 mA | PASS |
| 3.7 V | 0.1 Ω | 3.672 V | 3.659 V | 3.286 V | 4.995 V | 4.737 V | 277 mA | PASS |
| 3.7 V | 0.3 Ω | 3.616 V | 3.603 V | 3.286 V | 4.995 V | 4.737 V | 281 mA | PASS |
| 3.7 V | 0.5 Ω | 3.558 V | 3.544 V | 3.286 V | 4.995 V | 4.737 V | 285 mA | PASS |
| 3.3 V | 0.1 Ω | 3.269 V | 3.255 V | 3.286 V | 4.995 V | 4.737 V | 308 mA | PASS |
| 3.3 V | 0.3 Ω | 3.206 V | 3.191 V | 3.286 V | 4.995 V | 4.737 V | 314 mA | PASS |
| 3.3 V | 0.5 Ω | 3.140 V | 3.125 V | 3.286 V | 4.995 V | 4.737 V | 320 mA | PASS |
| 3.0 V | 0.1 Ω | 2.966 V | 2.950 V | 3.286 V | 4.995 V | 4.737 V | 338 mA | PASS |
| 3.0 V | 0.3 Ω | 2.896 V | 2.880 V | 3.286 V | 4.995 V | 4.737 V | 346 mA | PASS |
| 3.0 V | 0.5 Ω | 2.823 V | 2.806 V | 3.286 V | 4.995 V | 4.737 V | 355 mA | PASS |

Lowest steady stressed battery terminal voltage: **2.823 V** at 3.0 V OCV / 0.5 Ω.
Lowest steady stressed SYS voltage: **2.806 V** at 3.0 V OCV / 0.5 Ω.
Highest steady modeled battery current: **355 mA**.

The 3.0 V OCV cases sag below 3.0 V at the cell terminals. The simulated rails remain
valid, but a real protected cell/BMS may impose a higher practical cutoff. Treat the
3.0 V row as a regulator stress test, not a recommended discharge target.

These are system-level behavioral-model results. They do not validate switch current
limit, inductor saturation, thermal behavior, loop stability, ripple or EMI.
