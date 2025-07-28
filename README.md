# Inverted Pendulum Control Methods

This repository provides MATLAB 2024 implementations of various control strategies for the **inverted pendulum on a cart** system.  
The goal is to stabilize the pendulum in its upright position while controlling the cart position using different controllers.

---

## Repository Structure

```markdown
├── Controller_LQR.m
├── FLC
│   ├── Controller_Fuzzy_angle_simple_v0_2020.m
│   └── fuzzy_angle_position_simple_v169-Side.fis
├── Model Predictive Control
│   ├── Controller_MPC.m
│   └── Controller_MPC_CostFunction.m
├── Sliding Mode Control
│   └── Controller_Sliding_mode.m
│   └── SMC.pdf

---

## Contents

### LQR
- **Controller_LQR.m**  
  Linear Quadratic Regulator implementation for stabilizing the inverted pendulum.

### Fuzzy Logic Control
- **Controller_Fuzzy_angle_simple_v0_2020.m** – MATLAB script for fuzzy control.
- **fuzzy_angle_position_simple_v169-Side.fis** – Fuzzy inference system (rule base).

### Model Predictive Control
- **Controller_MPC.m** – MPC algorithm implementation.
- **Controller_MPC_CostFunction.m** – Defines the cost function used in the MPC optimization.

### Sliding Mode Control
- **Controller_Sliding_mode.m** – Sliding Mode Control implementation using a sliding surface and discontinuous control law.
- **SMC.pdf** - Derivation of the control law in the script

---

## Requirements
- MATLAB 2024 or later

---

## How to Use
1. Clone or download the repository.
2. Open MATLAB in the repository root.
3. Navigate to the desired controller folder (or file).
4. Run the `.m` script to simulate the system.
5. The simulation displays:
   - Position, angle, velocities
   - Control inputs
   - An animation of the cart-pendulum system.

---

## Notes
- Each folder contains all files necessary for that specific control strategy.
- Parameters can be tuned inside each script.
