# Robot Motion Kernel — MATLAB

ABB IRB1600 robot simulation implementing MoveJ and MoveL
motion kernels, integrated with an ABB RAPID program structure.

## Phases

| Phase | Description |
|---|---|
| 1 | MoveL proof-of-concept on simple test.urdf robot |
| 2 | Full RAPID path simulation on ABB IRB1600 (p10→p50) |
| 3 | Triangle drawing demo — equilateral 150mm path trace |

## Files

- `MoveL_phase1and2.m` — Main script, set PHASE = 1/2/3 and press F5
- `Lab_simulation.m`   — MoveJ simulation with full RAPID structure
- `test.urdf`          — Simple 6-DOF test robot
- `IRB1600/`           — STL mesh files for robot visualisation

## Requirements
- MATLAB R2020b or later
- Robotics System Toolbox

## How to Run
1. Set MATLAB working directory to this folder
2. Open `MoveL_phase1and2.m`
3. Set `PHASE = 1`, `2`, or `3` at the top
4. Press **F5**
