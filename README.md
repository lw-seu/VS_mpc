# Visual Servoing MPC for Autonomous UAV Pipeline Inspection in PX4 SITL simulation

> This is a quadrotor trajectory tracking control project based on a Model Predictive Control (MPC) framework, with disturbance rejection. It targets PX4 SITL simulation, but can be easily ported to real quadrotor platforms—especially when the flight controller is PX4. This project has been tested on Ubuntu20.04+ROS Noetic.

## Reference code
The code is based on the open-source project https://github.com/uzh-rpg/rpg_mpc?tab=readme-ov-file

## Install dependencies
git

MPC tool: ACADO toolkit is used, with qpoases as the solver. Installation reference: https://acado.github.io/install_linux.html

Eigen: required for matrix operations, version 3.4 or above. Installation reference: https://eigen.tuxfamily.org/index.php?title=Main_Page

PX4 firmware: it is recommended to use version 1.14.4: https://github.com/PX4/PX4-Autopilot/tree/v1.14.4

Dependencies required for simulation are detailed in the official PX4 documentation: https://docs.px4.io/v1.14/en/sim_gazebo_classic/

## Code structure
The code follows a bottom-to-top layered structure:

`mpc_controller`: generates the underlying ACADO solver code

`mpc_wrapper`: calls ACADO functions and wraps them as a low-level wrapper; it should not need changes (there shouldn’t be any bugs, right?). The related class is `MpcWrapper`

`mpc_ros_application`: a middle layer created for ROS-based implementation. The related class `MpcRosApplication` is a base class and cannot be instantiated; this class creates an instance of `MpcWrapper`

`mpc_tracking`: the actual application layer; most modifications should be made here. The related class `TrackingMpc` inherits from `MpcRosApplication`. To create new ROS-based code, simply add a new `.cpp` file and implement a subclass inheriting from `MpcRosApplication`; nothing else needs to be changed

`px4_interface`: PX4 interface layer, referring to the official code https://docs.px4.io/v1.14/en/ros/ , the related class is `OffboardMode`. If you create a new application-layer class, you need to modify it in the `OffboardMode` class

`tracking_node`: main function entry

## Running steps

1. Clone the project repository into the workspace:
   ```bash
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
   git clone https://github.com/lw-seu/quadrotor_mpc.git
   cd ~/catkin_ws
   ```

2. Build:
   ```bash
   catkin build quadrotor_mpc
   ```

3. Launch the PX4 node:
   ```bash
   roslaunch px4 mavros_posix_sitl.launch
   ```
   If it starts successfully, you will see Gazebo open and load the quadrotor correctly, and the terminal will display Ready to takeoff

   PS: On the first launch of the PX4 node, open the ground station `QGroundControl` and modify the parameter `THR_MDL_FAC` (0.8–1.0 should work) to make the thrust mapping closer to reality (see the Thrust Curve section in the official documentation https://docs.px4.io/v1.14/en/config_mc/pid_tuning_guide_multicopter.html). Later, if you change the maximum thrust constraint, it’s best to tune this parameter again

   <img width="583" height="107" alt="5215d4c4bb79865c448269a138b15031" src="https://github.com/user-attachments/assets/5ef3dc1a-75d9-41d6-b94c-fb1b0f847bce" />

5. Open another terminal and start the control algorithm node:
   ```bash
   rosrun quadrotor_mpc tracking_mpc
   ```
   The quadrotor flies three laps along a lemniscate trajectory, indicating successful execution

   
