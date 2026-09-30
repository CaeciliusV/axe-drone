# Axe Drone

A 5-inch, 4S quadcopter shaped like an axe, flown by a custom gesture-sensing hand controller.
Built as a team portfolio project for degree apprenticeship applications.

## Concept
- **Frame:** 3D-printed PETG-CF skeleton in a Y4 layout (two front motors + coaxial pair at the tail), with a printed axe shell and TPU bumpers
- **Electronics:** SpeedyBee F405 V5 stack, T-Motor Velox V3 2207 motors, 4S 2000mAh LiPo, Betaflight
- **Hand controller:** ESP32-S3 + LSM6DSV16X IMU in a printed handle, sending CRSF to a RadioMaster Ranger Nano (ExpressLRS)
- **Safety:** hardware arm switch + hold-to-fly button, speed capped at ~30mph

## Team
| Name | Role |
| --- | --- |
| Kosi | Electronics, Betaflight, CRSF firmware, integration lead |
| Mark | CAD + 3D printing (skeleton, shell, handle design) |
| Qasim | Hand controller lead (hardware + gesture code), test data, media |

## Repo layout
- `firmware/`: ESP32 hand controller code
- `cad/`: skeleton, shell and handle files
- `test-data/`: thrust stand + weight/CG data
- `documentation/`: wiring diagrams, notes, write-ups
- `media/`: photos and video

## Progress
- [ ] Parts ordered
- [ ] Motors spin on the bench
- [ ] Frame assembled
- [ ] Controller moves the sticks in Betaflight
- [ ] First hover
- [ ] First gesture-controlled flight
