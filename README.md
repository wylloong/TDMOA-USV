# TDMOA-USV

## Introduction

The capability of avoiding collisions in an environment with obstacles is a necessary function for an autonomous unmanned surface vehicle (USV).

We propose a novel multiple obstacles avoidance approach for USVs called the three-dimensional multiple obstacle avoidance (TDMOA) algorithm. This version releases the core implementation for TDMOA, containing seven modules:

- World model module, built from electronic charts and the dynamic local environment, can provide detailed local marine information for other modules;
- Navigation controller module, responsible for velocity control, course control and trajectory tracking, can steer the USV along the planned trajectory;
- Prediction module, assuming that obstacles proceed at their current statuses, can predicte obstacle's statuses (like speed, heading and position) with timestamp in a future time;
- COLREGs-based risk domain model module, inspired by the ship safety domain concept, can quantify the threat/rule cost with maritime rules;
- 3D risk model module, introducing the time dimension to the marine plane, can establish a 3D risk model with the predictive risk domains and represent the local movement trend of multiple obstacles and potential hazards to the USV in the future time.
- Path search module, a improved D star lite algorithm, can search an optimal local obstacle avoidance path in cycles;
- Simulation module, integrated with a custom autonomy suite, can validate the feasibility and effectiveness of our proposed approach in tackling the multi-obstacle avoidance problem;

## Simulation demo
### Snapshots
![Initial Stage](https://github.com/wylloong/TDMOA-USV/blob/main/images/demo_1.png)

![Overtaking and Crossing scene](https://github.com/wylloong/TDMOA-USV/blob/main/images/demo_2.png)
