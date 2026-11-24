# TUC.io Drifters – Bosch Future Mobility Challenge 2026

Vehicle-side software of the **TUC.io Drifters** team (SenseLab Space Informatics, Technical University of Crete) for the **Bosch Future Mobility Challenge 2026**, where the team reached the **global final and placed 7th out of 163 teams**.

The car is a 1:10 scale autonomous vehicle built around a Raspberry Pi 5 and an STM32 Nucleo motor controller, with an IMU, ToF sensors and a Luxonis OAK-D Lite stereo camera.

The complete software lives in [`Brain/`](Brain) (see [`Brain/README.md`](Brain/README.md) for the architecture, setup and usage). Highlights:

- Lane and road-marking detection, object recognition (neural network deployed on the OAK-D)
- Stanley and PID controllers for lane keeping
- Extended state machine for the competition scenarios (crosswalks, roundabouts, highway, parking)
- Global and local localisation, track-aware path planning
- Angular dashboard for monitoring and calibration
- Sensor and actuator communication (IMU, ToF, cameras, Nucleo)

## Authorship and credits

This is **team work, and it is not the work of a single person.**

- The base of the software is the official **BFMC Brain** by Bosch Engineering Center Cluj and the BFMC organisers, released under the BSD 3-Clause license (see [`Brain/LICENSE`](Brain/LICENSE)). Its copyright notice is kept unchanged.
- The 2026 extensions (perception, control, decision-making, localisation, planning, dashboard) were developed by the TUC.io Drifters team members.
- This repository is a single snapshot of the final code, so the original commit history, and with it the per-person attribution, is not part of it.

## My role

I joined the team as a **mechatronics engineer**. My main contribution was on the hardware side: selecting the components, designing the 3D-printed sensor mounts in Fusion 360, and the physical installation, calibration and data pipeline of the IMU (BNO085), Time-of-Flight sensors and the OAK-D Lite stereo camera. I also took part in the work on the decision-making logic and the lane-keeping controllers, but the larger share of the code was written by other team members.

*Panagiotis Mantouvalos* · [LinkedIn](https://www.linkedin.com/in/panos-mantouvalos/)
