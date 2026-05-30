# ERC Propulsion

Repository for Propulsion of a Rover for competing in ERC, including a Kinematic model and ODrive-based hub-motor control for ROS2.
This project contains utility scripts, configuration files, and a ROS2 package.
(`propulsion_ros`) to control implement a kinematic model and control node to actuate multiple ODrive and Steering axes.

## Contents 
- `config.json` — default hoverboard ODrive settings (for H1/H2 motor).  
- `calibrate.py` — helper script for calibration steps (use on a free-spinning motor only).
- `multi_velocity.py` — simple script to test multiple drives with velocity commands.
- `teleplot_odrive.py` — plots ODrive telemetry (works with Teleplot VSCode extension).
- `propulsion_ros/` — ROS2 Python package with multiple packaged nodes, for kinematics, odrive control and mission control.

## Prerequisies
- ODrive v3.6 (and derivatives) with [firmware version 0.5.6](https://docs.odriverobotics.com/releases/firmware), others not supported
- [Update using STM32CubeProgrammer](https://ffbeast.github.io/docs/en/software_firmware_flashing.html). Hold BOOT then press RESET, then connect over USB
- ODrive package: `pip install odrive` (currently using version 0.6.10.post0)

## Quick start
1. Connect the ODrive to your PC (USB) and confirm it appears in `odrivetool`. Note the serial number!  
2. For a new motor other than the 6" H1/H2, follow the hoverboard setup guide. Note that for MKS Odrive, instead of `odrv0` it will appear as `dev0` due to it not being a genuine device. Otherwise, exit and restore the config:  
```bash
odrivetool restore-config config.json
```
3. Run calibration (set serial numbers of motors to calibrate first!)
```bash
python calibration.py
```
4. Run the simple velocity tester =:
```bash
python velocity.py
```
5. Test multiple motors (must set serial numbers of connected motors) =:
```bash
python multi_velocity.py
```

## ROS usage and visualization
The repository includes a ROS2 package in `propulsion_ros/` which exposes 
three nodes `mission_control`, `teleop` and `odrive`.
- `mission_control` - Converts controller commands into a geo-msg Twist which is actionable by the teleoperation/kinematics node.
- `teleop` - Teleoperations and Kinematic node, takes a Twist and send the velocity and rotation commands to both steering and odrive propulsion.
- `odrive` - Initialises the odrives, and takes velocity commands to actuate. Then returns encoder information to teleop in order to solve inverse kinematics.

## Kinematic Model
If we take a control vector $$u$$, wheel velocity vector $$\Phi$$ and Kinematic Matrix $$K$$:
```math
u = \begin{bmatrix}
\dot x  \\ \dot y \\ \dot \\ \dot \theta
\end{bmatrix} \; \;
\Phi = \begin{bmatrix}
\dot \phi_{1} \\
\dot \phi_{2} \\
\dot \phi_{3} \\
\dot \phi_{4}
\end{bmatrix} \; \;
K = \begin{bmatrix}
\cos(\theta_{1}) && \cos(\theta_{2}) && \cos(\theta_{3}) && \cos(\theta_{4}) \\
\sin(\theta_{1}) && \sin(\theta_{2}) && \sin(\theta_{3}) && \sin(\theta_{4}) \\
\frac{sin(\theta_{1} - \alpha_{1})}{l_{1}} && \frac{\sin(\theta_{2} - \alpha_{2})}{l_{2}}
&& \frac{\sin(\theta_{3} - \alpha_{3})}{l_{3}} && \frac{\sin(\theta_{4} - \alpha_{4})}{l_{4}} \\
\end{bmatrix}

```
Where $$\theta$$ is the angle of each wheel from the forward direction, $$\alpha$$ is the angle of each wheel base from the center of mass/rotation and $$l$$ is the distance
from the center of mass/rotation of each wheel base. All angles are counter-clockwise (ccw).

It can be shown that they are related by the equation:
``` math
u = \dfrac{K \cdot \Phi}{4}
```
One can find the required wheel velocities $$\Phi$$ to match the control vector $$u$$ by using the Moore-Penrose (Pseudoinverse):
``` math
\begin{equation}
    K^+ = K^T (K K^T)^{-1}
\end{equation}
```
Further details can be found in the original paper - [Kinematic Model](https://www.cambridge.org/core/journals/robotica/article/multiconfiguration-kinematic-model-for-active-drivesteer-fourwheel-robot-structures/FC9654B111F8B3954BF505E2B957F003?utm_campaign=shareaholic&utm_medium=copy_link&utm_source=bookmark)

## Links and documentation
- [ODrive firmware v0.5.6 docs (DON'T USE LATEST!)](https://docs.odriverobotics.com/v/0.5.6/)
- [Hoverboard setup (follow for new motors!)](https://docs.odriverobotics.com/v/0.5.6/hoverboard.html)
- [API Documentation](https://docs.odriverobotics.com/v/0.5.6/fibre_types/com_odriverobotics_ODrive.html)
- [Tuning Guide](https://docs.odriverobotics.com/v/0.5.6/control.html )
- [Kinematic Model](https://www.cambridge.org/core/journals/robotica/article/multiconfiguration-kinematic-model-for-active-drivesteer-fourwheel-robot-structures/FC9654B111F8B3954BF505E2B957F003?utm_campaign=shareaholic&utm_medium=copy_link&utm_source=bookmark)

## Safety and tuning notes

- Use `odrivetool` to limit currents, voltages and speeds; sensible limits are
	already stored in `config.json`.
- Important limits to consider:
	- max DC input current (protect PSU/battery)
	- max DC output current (regen braking)
	- max motor current (thermal safety)
	- max speed and max DC voltage
- The motor thermistor should ideally be configured to trigger overheat
	shutdown: https://docs.odriverobotics.com/v/0.5.6/thermistors.html#thermistor-coefficients  
- Position-control requires calibrated motors
