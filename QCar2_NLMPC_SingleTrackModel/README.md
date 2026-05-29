# QCar2_NLMPC_SingleTrackModel

This project is the Single Track Model version of the QCar2 NLMPC project.

## Model

State vector:

```text
x = [X, Y, psi, vx]^T
```

Input vector:

```text
u = [delta, ax]^T
```

Kinematic single-track dynamics:

```text
X_dot   = vx cos(psi)
Y_dot   = vx sin(psi)
psi_dot = vx / Lwb * tan(delta)
vx_dot  = ax
```

## Main commands

```bash
source setup_env.sh
python scripts/generate_trajectory.py
python scripts/build_solver.py
python scripts/qcar2_nlmpc.py
```

## Configuration

Edit `config/qcar2_nlmpc.yaml` for model parameters, constraints, MPC weights, and simulation initial conditions.

For a reference-based start:

```yaml
simulation:
  initial_state: reference
  initial_input: reference
```

For a custom start:

```yaml
simulation:
  initial_state: [X0, Y0, psi0, vx0]
  initial_input: [delta0, ax0]
```
