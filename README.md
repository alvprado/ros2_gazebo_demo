# ROS 2 Modular Robot Simulation Demo

A modular closed-loop control and simulation stack for a differential-drive robot,
built with ROS 2, Gazebo Harmonic, and modern C++.

The project demonstrates a backend-agnostic controller architecture: the same control
stack runs unchanged against either a custom lightweight simulator or a Gazebo-based
physics simulation. Any backend exposing the same `/cmd_vel` and `/odom` interface can
be integrated through an infrastructure adapter, including real hardware.

## Features / TL;DR

- Closed-loop control of a differential-drive robot in ROS 2.
- Swappable simulation backends: a custom first-order simulator and Gazebo Harmonic.
- Same controller interface for both backends: `/odom` in, `/cmd_vel` out.
- Curvature-based steering: target curvature is converted into an angular velocity setpoint
- ROS-independent domain logic for the controller, simulator, filters, target generation, and integrators.
- Modern C++ design using `std::expected`, C++20 concepts, templates, and dependency injection.
- Unit-testable core logic without launching ROS, Gazebo, or RViz.
- RViz visualization using URDF, joint states, and TF.

## Architecture

![Architecture](/docs/arch_diagram.svg)

The controller subscribes to `/odom` and publishes to `/cmd_vel`. It has no knowledge
of which backend is running. Swapping backends requires no changes to the control stack as long as the backend exposes the same `/cmd_vel` and `/odom` interface. A real robot could be integrated through the same interface using an appropriate hardware adapter.

## Packages

### `controller`

The robot control stack. Fully split into domain and infrastructure layers.

**Domain (no ROS dependencies):**

| Component | Description |
|-----------|-------------|
| `TargetsGenerator` | Step-profile generator. Takes a list of `(value, duration_s)` pairs and returns the current target at any query time. Supports looping. Used for both longitudinal velocity and curvature targets. |
| `PidController<SetpointFilter, DerivativeFilter>` | Generic PID with feedforward. Setpoint and derivative filters are dependencies injected via templates, constrained by C++20 concepts, making the filter type a compile-time policy. Output is clamped to configurable limits. Returns `std::expected<double, PidErrorCodes>`. |
| `CurvatureController<ControlLaw>` | Converts a target curvature and current longitudinal velocity into an angular velocity command using the relation $w = \kappa v $. Falls back to a configurable spin-in-place or straight line mode when the robot is nearly stopped to avoid division-by-near-zero. The underlying control law is a dependency injected via templates and validated by C++20 concepts. Returns `std::expected<double, CurvatureControllerErrorCodes>`.|
| `LowPassFilter` | First-order IIR filter. Used to smooth the PID setpoint and derivative term. |

**Infrastructure:**

| Node | Subscribes | Publishes |
|------|-----------|-----------|
| `ControllerNode` | `/odom`, `/target_longitudinal_velocity`, `/target_curvature` | `/cmd_vel` |
| `TargetsGeneratorNode` | — | `/target_longitudinal_velocity`, `/target_curvature` |

**Parameters:**

| File | Contents |
|------|----------|
| `config/controller_params.yaml` | PID gains, setpoint and derivative filter cutoff frequencies, output limits, lower velocity threshold value for spin-in-place mode; also `control_frequency`. |
| `config/targets_generator_params.yaml` | Target velocity and curvature sequences as parallel `values[]` / `durations_s[]` arrays, `targets_update_frequency_hz`, and `loop` flag. |

### `simulation`

The simple simulation backend plus the launch files for both backends.

**Domain (no ROS dependencies):**

| Component | Description |
|-----------|-------------|
| `RobotModel` | Robot is modeled as a simple first-order system on both longitudinal and angular velocity. It receives a linear and angular velocity command and translates them into a 2D velocity and pose plus wheel angle states. The integration scheme is a pluggable function object, allowing Forward Euler, Heun, or Runge-Kutta to be substituted without changing the model. Returns `std::expected<State, SimulationErrorCodes>`. |
| `forwardEulerStep<State, Command, Derivative>` | Templated integrator constrained by the `NumericIntegrable` concept. It takes the current state, the given command, a callable to the model dynamics (`f(x,u)`) and a time step to compute the next state.|

**Infrastructure:**

| Node | Subscribes | Publishes |
|------|-----------|-----------|
| `SimulatorNode` | `/cmd_vel` | `/odom`, `/joint_states`, `/tf` (odom → base\_link) |

**Parameters:**

| File | Contents |
|------|----------|
| `config/simulation_params.yaml` | First-order lag time constants for longitudinal and angular velocity, wheel radius and separation, and `simulation_frequency`. |

### `robot_description`

The robot is a simple two-wheeled differential drive platform with a passive frictionless caster wheel at the front for balance. Its geometry is defined in a URDF xacro file.

The two backends use the URDF differently. The Gazebo backend reads it directly to spawn the robot, so the URDF is the single source of truth for geometry and physics. The simple simulator reads the robot geometry from its parameter file `simulation_params.yaml` independently; the URDF is used only for visualization. This means both files must be kept in sync manually when geometry changes.

## Robot Dynamics Model

The simple simulator uses the following robot dynamics model:

$$ \dot{\bf{x}} = \bold{f}(\bold{x},\bold{u}) = \begin{bmatrix} \dot{x} \\
\dot{y} \\
\dot{\theta} \\
\dot{v} \\
\dot{\omega} \\
\dot{\phi}_{\text{left}} \\
\dot{\phi}_{\text{right}}
\end{bmatrix} = \begin{bmatrix} v \cos (\theta) \\
v \sin(\theta) \\
\omega \\
\frac{(v_{\text{cmd}} - v)}{T_{\text{long}}} \\
\frac{(\omega_{\text{cmd}} - \omega)}{T_{\text{angular}}} \\
\frac{v - \frac{\omega L}{2}}{R} \\
\frac{v + \frac{\omega L}{2}}{R}
\end{bmatrix}
$$

where $x$, $y$ and $\theta$ are the robot's planar pose, $v$ and $\omega$ the linear and angular velocity and $\phi_{\text{left}}, \phi_{\text{right}}$ the left and right wheel angles. The commands -- instead of low-level actuator torques -- are given in high-level linear and angular velocity commands $v_{\text{cmd}}$ and $\omega_{\text{cmd}}$. The linear and angular velocity states are hence modeled as first-order systems w.r.t. their corresponding commands and with the time constants $T_{\text{long}}$ and $T_{\text{angular}}$. $L$ is the wheel separation and $R$ describes the wheel radius.

## Controller Design

The control stack consists of two feedback loops for longitudinal and angular velocity:

![Control](/docs/control_diagram.svg)


**Longitudinal loop:** a PID with feedforward tracks the target linear velocity directly.

**Angular loop:** the `CurvatureController` converts the target curvature $\kappa$ to a
target angular velocity $\omega = \kappa v$ using the current linear velocity. This formulation expresses the desired turn as a geometric curvature rather than as a fixed angular velocity, so the commanded angular velocity scales with the robot’s current speed. A separate PID then closes the loop on angular
velocity.

Both PIDs use configurable first-order IIR filters on the setpoint and derivative term
to reduce noise sensitivity.


## Building

```bash
# From the workspace root
colcon build
source install/setup.bash
```

Run the unit tests:

```bash
colcon test
colcon test-result --all
```

## Running

### Simple simulator (fast iteration)

```bash
ros2 launch simulation launch_simple_sim.py
```

Starts the first-order lag simulator, both controller nodes, and RViz.

### Gazebo backend (physics-based simulation)

```bash
ros2 launch simulation launch_gazebo_sim.py
```

Starts gz-sim headless, spawns the robot, brings up the ROS–Gazebo bridge,
and launches the controller nodes (delayed 6 s to allow the robot to settle
on the ground before receiving velocity commands).

## Design Decisions

### Clean Architecture

Domain code (`RobotModel`, `PidController`, `CurvatureController`, `TargetsGenerator`)
has zero ROS dependencies. This means:

- All domain logic is unit-testable without spinning up a ROS node or a simulator.
- Swapping the simulator backend only requires a new infrastructure adapter that
  publishes `/cmd_vel` and `/odom`.

### `std::expected` for error propagation

All fallible domain operations return `std::expected<T, ErrorCode>` rather than
throwing or returning sentinel values. This makes error paths explicit at the call
site and keeps domain code exception-free.

### C++ concepts for policy types

`PidController`, `CurvatureController`, and `forwardEulerStep` use C++20 concepts to
constrain their template parameters (`Filter`, `ControlLaw`, `NumericIntegrable`).
This documents the required interface at the declaration site and produces clear
compiler errors when the contract is violated, rather than obscure deep-template
failures.

### Pluggable integration scheme

The integrator is injected into `RobotModel` as a `std::function` rather than being
hard-coded. Forward Euler is used by default, but switching to a higher-order scheme
(Heun, RK4) requires no changes to the model itself — only the function passed at
construction.
