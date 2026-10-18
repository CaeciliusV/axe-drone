Betaflight Modes - tested 18 Oct 2026
- ARM on AUX1 1700-2100: switch reaches FC (AUX1 = 2000 when armed, 1000 when not): PASS
- ANGLE on AUX1 900-2100 (always on): PASS
- Shows "ARM (DISABLED)" while configurator connected / no battery - expected safety lock
- IMU must be chip-side up at boot (upside down causes roll to snap 1100/1900)
