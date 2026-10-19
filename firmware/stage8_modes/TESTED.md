Stage 7 + 8 - tested 19 Oct 2026
Stage 7 (gyro + accel complementary filter, ALPHA 0.98):
- Fused angle much steadier than accel-only; table slam: acc jumps to double digits, fused barely moves: PASS
- Gyro sign check (GX_SIGN / GY_SIGN = 1.0): PASS
Stage 8 (trick button control system):
- Hold trick = yaw (tilt L/R -> yaw, roll held 1500): PASS
- Tap trick = NORMAL <-> PAUSE, PAUSE: sticks + throttle frozen: PASS
- In PAUSE tap climb/descend = TRICK (rising beeps), throttle works in TRICK: PASS
- Tap trick in TRICK = NORMAL: PASS
- Kill combo in PAUSE disarms, no switch to TRICK: PASS
- Hold trick in PAUSE = no yaw: PASS
- Rocker OFF in any mode = disarm + reset: PASS
Controller now runs fully wireless from 2S battery via power harness + buck.
