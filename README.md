# 2D Inverse Dynamics Gait Analysis

Estimating the **net joint moments** and **intersegmental reaction forces** at the ankle, knee, and hip during a human gait cycle, from planar motion-capture markers and force-plate data, using a recursive Newton–Euler inverse-dynamics formulation in MATLAB.

![Net joint moments during the gait cycle](results/moments.png)

---

## Overview

When a person walks, we can measure where their joints are (with reflective markers and motion-capture cameras) and how hard the ground pushes back (with a force plate). What we *can't* directly measure are the internal torques the muscles and joints generate to produce that motion. **Inverse dynamics** recovers those internal loads from the measured kinematics and ground reaction forces.

This project implements a 2D (sagittal-plane) inverse-dynamics pipeline for the stance-and-swing of a single leg. Given marker trajectories for the hip, knee, fibula, ankle, and metatarsal head, plus ground reaction force and center-of-pressure data, it computes the time history of:

- net **moment** at the ankle, knee, and hip, and
- the **reaction force** transmitted across each joint.

The subject is modeled as three rigid segments — **foot, shank (leg), and thigh** — with masses, lengths, and moments of inertia scaled from total body mass using standard anthropometric ratios.

## Method

The analysis proceeds **distal to proximal** (foot → shank → thigh). Each segment is treated as a rigid body, and Newton–Euler equations are solved at every time frame:

$$\sum \vec{F} = m\,\vec{a}_{\text{com}}, \qquad \sum M_{\text{com}} = I\,\alpha$$

For each segment the pipeline:

1. **Builds segment geometry** — computes the segment orientation from its two markers via `atan2`, and locates the center of mass along the segment using anthropometric fractions of its length.
2. **Differentiates kinematics** — applies central finite differences to obtain linear and angular velocities and accelerations of each segment's center of mass.
3. **Applies the equations of motion** — solves the force balance for the proximal joint reaction force, then the moment balance about the center of mass for the net joint moment.
4. **Passes loads upward** — the reaction force and moment at the proximal joint of one segment become the known distal loads of the next segment up the chain. The ground reaction force enters at the foot as the only externally measured load.

Marker noise is amplified by differentiation, so a centered moving-average filter is applied to the raw coordinates and again after each derivative to keep the acceleration estimates usable.

## Results

**Net joint moments.** The ankle shows the characteristic large plantarflexor moment that builds through mid-to-late stance — the "push-off" that propels the body forward — while the knee and hip moments are smaller and reverse sign across the cycle.

![Net joint moments](results/moments.png)

**Intersegmental reaction forces.** The vertical components ($F_y$) at all three joints reproduce the classic **double-bump (M-shaped) loading pattern** of walking: a peak at weight acceptance and a second at push-off, with a dip during midstance.

![Intersegmental joint reaction forces](results/forces.png)



## Repository structure

```
.
├── src/
│   ├── gait_analysis.m   # main inverse-dynamics pipeline (run this first)
│   └── frames.m          # stick-figure animation from the computed posture
├── data/
│   └── Project2Data.mat  # marker + force-plate data (106 frames, ~70 Hz)
├── results/              # figures rendered from the analysis
└── README.md
```

## How to run

Requirements: **MATLAB** (developed and tested on a recent release; no add-on toolboxes required — `movmean` and `unwrap` are in base MATLAB).

```matlab
% from the repository root, or with src/ on the path
run('src/gait_analysis.m')   % computes loads and produces the moment/force plots
run('src/frames.m')          % then renders the stick-figure animation
```

`gait_analysis.m` loads the data via a path relative to the script, so it runs correctly regardless of the current working directory. `frames.m` reuses the workspace variables created by `gait_analysis.m`, so run it second in the same session.

## Data

`Project2Data.mat` contains two matrices:

- **`MarkerData`** — 106 frames × 18 columns: frame index, time stamp, and the $(x, y)$ coordinates of the hip, knee, fibula, ankle, and metatarsal markers (meters).
- **`ForceData`** — 106 frames × 5 columns: ground reaction force components and center of pressure from the force plate.

The trial spans ~1.5 s sampled at roughly 70 Hz.

## Modeling assumptions & limitations

- **Planar (2D) analysis** — motion is assumed to lie in the sagittal plane; out-of-plane rotation is ignored.
- **Rigid segments** with masses, COM locations, and inertias from population-average anthropometric ratios rather than subject-specific measurement.
- **Finite-difference differentiation** of noisy marker data; results depend on the smoothing window, which is a tunable parameter.
- A single force plate is assumed to capture the full ground reaction for the analyzed limb.

## Possible extensions

- Subject-specific anthropometry or a Savitzky–Golay / Butterworth filter in place of the moving average.
- Normalization to % gait cycle and overlay of multiple trials with confidence bands.
- Joint **power** ($M \cdot \omega$) to distinguish energy generation from absorption at each joint.
- Extension to a 3D or full lower-limb (including pelvis) model.

## Author

Peter Ziegler
