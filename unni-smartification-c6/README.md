# Rev A v11 — low-cost power rework

Replaced BQ24074 / TPS61023 / TPS2116 concept with:

- MCP73812T-420I/OT (~303 mA via 3.3 kΩ PROG), CE controlled by ESP and default-disabled by 100 kΩ pull-down.
- Discrete load sharing: USB -> SS14 -> SYS; VBAT -> AO3401A -> SYS, gate controlled from USB.
- TPS63031 retained as always-on 3.3 V buck-boost.
- TPS613222ADBZ fixed 5 V boost, hard-switched with AO3401A/AO3400A.
- Second AO3400A provides hardware USB-present inhibit.
- Two AP22913 reverse-blocking load switches OR USB and boost into UNNI_AC.
- Added GPIO-excited 10 k NTC battery temperature divider + 100 nF filter.

Important: AP22913 uses KiCad's AP22913CN4 logical symbol but value/footprint target AP22913W6-7 SOT-26. Pin mapping must be verified when assigning the final footprint.
