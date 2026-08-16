# Native API runtime diagnostic

Temporary instrumentation for the intermittent state where Wi-Fi/IP remain
reachable while the ESPHome Native API stops returning entity states and pongs.

The component logs once per second:

- component loop count;
- maximum gap between component loop invocations;
- maximum time spent inside the component loop;
- maximum time spent in history, power-policy, HA publication, battery,
  RT/RH, CO2 and power-save stages.

Every 10 seconds it also logs free heap, largest free 8-bit block and minimum
free heap seen since boot.

The diagnostic does not change GPIO ISR handlers or RT/RH/I2C capture logic.
