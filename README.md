# Visual Servoing MPC for Autonomous UAV Pipeline Inspection in PX4 SITL simulation

[![ubuntu](https://img.shields.io/badge/ubuntu-20.04-orange?logo=ubuntu)](https://releases.ubuntu.com/focal/)
![ROS](https://img.shields.io/badge/ROS-noetic-blue?&logo=ROS)
![PX4](https://img.shields.io/badge/px4%20firmware-1.14.x-blue.svg)
![gazebo](https://img.shields.io/badge/gazebo%20classic-11.x.x-orange)
[![eigen](https://img.shields.io/badge/eigen-3.4.x-green)](https://libeigen.gitlab.io/)
![c++](https://img.shields.io/badge/C++11-blue)

## Introduction
This project uses an autonomous quadrotor pipeline inspection framework based on image-based visual servoing model predictive control (VMPC) for 3D inspection scenarios.  To enhance robustness under low-rate visual measurements and environmental uncertainties, an extended-state Kalman filtering scheme with image feature prediction (ESKF-PRE) is developed,and the estimated lumped disturbances are incorporated into the VMPC prediction model, resulting in the proposed ESKF-PRE-VMPC framework. The detail algorithm can be found at 


![vs_mpc_x8](https://github.com/user-attachments/assets/c76c8bb7-5e42-48e4-a017-b491f41316b3)




## Reference code
The code is based on the open-source projects https://github.com/uzh-rpg/rpg_mpc?tab=readme-ov-file and https://github.com/EPVelasco/VisualServoing_NMPC.

## Install dependencies
git

MPC tool: ACADO toolkit is used, with qpoases as the solver. Installation reference: https://acado.github.io/install_linux.html

Eigen: required for matrix operations, version 3.4.x. Installation reference: https://eigen.tuxfamily.org/index.php?title=Main_Page

PX4 firmware: it is recommended to use version 1.14.4: https://github.com/PX4/PX4-Autopilot/tree/v1.14.4

Dependencies required for simulation are detailed in the official PX4 documentation: https://docs.px4.io/v1.14/en/sim_gazebo_classic/

## Code structure
The code follows a bottom-to-top layered structure:

`mpc_controller`: generates the underlying ACADO solver code

`mpc_wrapper`: calls ACADO functions and wraps them as a low-level wrapper; it should not need changes. The related class is `MpcWrapper`

`mpc_ros_application`: a middle layer created for ROS-based implementation. The related class `MpcRosApplication` is a base class and cannot be instantiated; this class creates an instance of `MpcWrapper`

`pipeline_inspection`: the actual application layer. The related class `PipelineInspection` inherits from `MpcRosApplication`. 

`px4_interface`: PX4 interface layer, referring to the official code https://docs.px4.io/v1.14/en/ros/ , the related class is `OffboardMode`. If you create a new application-layer class, you need to modify it in the `OffboardMode` class

`tracking_node`: main function entry
## Preparation Before Running

1. Copy the required Gazebo resources

   * Place the `pipeline.world` file (located in the `gazebo/world` directory of this source code) into:

     ```
     ~/.gazebo/worlds
     ```

   * Place the `gas_pipeline` folder (located in the `gazebo/model` directory of this source code) into:

     ```
     ~/.gazebo/models
     ```

   These directories should exist after installing Gazebo. If they do not exist, please create them manually.


2. Add a new PX4 launch file

   * Navigate to the `launch` directory inside the PX4 source code `PX4-Autopilot`.

   * Create a new file named:

     ```
     vsmpc_mavros_sitl.launch
     ```

   * Copy the contents of `mavros_posix_sitl.launch` into the newly created file.

   * Modify the file as follows:
     <img width="1117" height="51" alt="图片" src="https://github.com/user-attachments/assets/94b96223-9c48-4843-8192-8bda3b2591e3" />
     ⬇⬇⬇⬇⬇⬇
     <img width="1150" height="78" alt="图片" src="https://github.com/user-attachments/assets/52e2952a-d206-414c-9860-57323dbc3889" />




3. Configure environment variables

   Add the following lines to your `~/.bashrc` file:

   ```bash
   source ~/ACADOtoolkit/build/acado_env.sh

   source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
       ~/PX4-Autopilot \
       ~/PX4-Autopilot/build/px4_sitl_default

   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/.gazebo/models
   ```

   After editing the file, apply the changes by running:

   ```bash
   source ~/.bashrc
   ```



## Running steps

1. Clone the project repository into the workspace:
   ```bash
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
   git clone https://github.com/lw-seu/VS_mpc.git
   cd ~/catkin_ws
   ```

2. Build:
   ```bash
   catkin build visual_servo_mpc
   ```

3. Start the control algorithm node:
   ```bash
   rosrun quadrotor_mpc inspection_mpc
   ```
   
4. Open another terminal and launch the PX4 node:
   ```bash
   roslaunch px4 vsmpc_mavros_sitl.launch
   ```
   If it starts successfully, you will see Gazebo open and load the quadrotor and the pipeline model correctly.
> [!NOTE]
> On the first launch of the PX4 node, open the ground station `QGroundControl` and modify the parameter `THR_MDL_FAC` (0.8–1.0 should work) to make the thrust mapping closer to reality (see the Thrust Curve section in the official documentation https://docs.px4.io/v1.14/en/config_mc/   pid_tuning_guide_multicopter.html). Later, if you change the maximum thrust constraint, it’s best to tune this parameter again

<img width="583" height="107" alt="5215d4c4bb79865c448269a138b15031" src="https://github.com/user-attachments/assets/5ef3dc1a-75d9-41d6-b94c-fb1b0f847bce" />

**If the simulation runs successfully, the UAV will autonomously fly along the pipeline in Gazebo, and a `result` window will appear showing the real-time image detection output.**

   
