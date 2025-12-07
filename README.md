# BayesFilter-Gas-Mapping (ROS2 Jazzy, C++)

A unique ROS2 C++ mini-project implementing a custom Bayes Filter (from scratch) to estimate the robot’s position in a 2D grid using simulated gas sensor measurements generated from a Gaussian Plume Model. Includes full mathematical explanation, RViz visualization, and TurtleBot3 Gazebo simulation.

---

## 🚀 Project Overview

This project demonstrates:

* A Discrete Grid Bayes Filter implemented manually in C++
* Motion + Sensor updates derived from full probability equations
* Gaussian Plume Gas Sensor simulation
* Visualization using OccupancyGrid in RViz
* TurtleBot3 simulation in Gazebo (ROS2 Jazzy)

Useful for showcasing probabilistic robotics skills for master's applications.

---

## 📦 Prerequisites

### 1. Install ROS2 Jazzy

Follow the official setup guide.

### 2. Clone TurtleBot3 dependencies (Jazzy branches)

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
```

### 3. Install dependencies via rosdep

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
```

### 4. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

## ▶️ Running the Project

Open **5 terminals** and execute the following commands **in each terminal**:

### Terminal 1 – Start Gazebo with TurtleBot3

```bash
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Terminal 2 – Gas Plume + Virtual Sensor Simulator

```bash
ros2 run gas_bot_cpp gas_simulator
```

### Terminal 3 – Bayesian Filter Node (the magic happens here)

```bash
ros2 run gas_bot_cpp bayes_filter
```

### Terminal 4 – Drive the Robot with Keyboard

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Keyboard Controls:**

* `W` = forward
* `X` = backward
* `A` = turn left
* `D` = turn right
* `S` = stop

### Terminal 5 – RViz2 Visualization

```bash
ros2 run rviz2 rviz2
```

**RViz2 Setup (do this once):**

* Global Options → Fixed Frame → `odom`
* Add → By topic:

  * `/belief_grid` → OccupancyGrid → Properties → Color Scheme → map → costmap (rainbow mode!)

    * Blue/Green = low probability
    * Yellow → Red → Pink = source is here!
  * `/robot_description` → RobotModel
  * Optional: `/scan` → LaserScan, `/odom` → Path

---

Drive the robot around and watch the belief heatmap converge on the hidden gas source in seconds!


---

## 🧠 Mathematical Foundation

### 1. State Space

The environment is represented as a discrete grid of cells:

$$
\mathcal{X} = \{x_1, x_2, \dots, x_N\}
$$

Each cell $x_i$ is associated with a belief (probability) $\text{bel}(x_i)$ that the source is located there.
Initially, a uniform prior is typically used: $\text{bel}(x_i) = \frac{1}{N}$.

### 2. Motion Update (Prediction Step)

When the robot/platform executes a control/action $u_t$, the belief is propagated forward:

$$
\overline{\text{bel}}(x_i) = \sum_{j} p(x_i \mid x_j, u_t) \, \text{bel}(x_j)
$$

We model motion uncertainty with a Gaussian distribution:

$$
p(x_i \mid x_j, u_t) = \mathcal{N}(x_i; \, x_j + u_t, \, \Sigma_{\text{motion}})
$$

(where $\Sigma_{\text{motion}}$ is the motion covariance matrix).

### 3. Measurement Model (Gaussian Plume Sensor Model)

The expected concentration at location $x_i = (x, y, z)$ downwind from a continuous point source of strength $Q$ is given by the classic Gaussian plume formula (simplified, assuming wind along x-axis):

$$
\hat{z}(x_i) = \frac{Q}{2\pi \sigma_y \sigma_z \, u} \exp\left(-\frac{y^2}{2\sigma_y^2}\right) \exp\left(-\frac{(z - H)^2}{2\sigma_z^2}\right)
$$

where:
- $u$ = mean wind speed
- $\sigma_y, \sigma_z$ = dispersion coefficients (functions of downwind distance $x$)
- $H$ = effective source height

The measurement likelihood is modeled as log-linear (common for chemical sensors):

$$
p(z_t \mid x_i) = \exp\left( -k \, (z_t - \hat{z}(x_i))^2 \right)
$$

or more commonly with a proper Gaussian:

$$
p(z_t \mid x_i) = \mathcal{N}(z_t; \, \hat{z}(x_i), \, \sigma_{\text{sensor}}^2)
$$

(choose one based on your sensor characteristics).

### 4. Measurement Update (Correction Step)

The posterior belief after incorporating measurement $z_t$ is:

$$
\text{bel}(x_i) = \eta \, p(z_t \mid x_i) \, \overline{\text{bel}}(x_i)
$$

with the normalizer

$$
\eta = \left[ \sum_i p(z_t \mid x_i) \, \overline{\text{bel}}(x_i) \right]^{-1}
$$

This completes one cycle of the Bayes filter (grid-based recursive Bayesian estimation).

### Summary of the Algorithm (per time step)

1. **Prediction**: Compute $\overline{\text{bel}}(x_i)$ using motion model
2. **Correction**: Update $\text{bel}(x_i) \leftarrow \eta \, p(z_t \mid x_i) \, \overline{\text{bel}}(x_i)$
3. (Optional) Resample or renormalize if needed
4. Source location estimate: $\hat{x} = \arg\max_i \, \text{bel}(x_i)$ or expected value


## 🎨 RViz Visualization

The belief grid is mapped to OccupancyGrid [0–100]:

```
value = (belief[i] / max_belief) * 100
```

High probability → bright cells.

---

## 📁 Repository Structure

```
bayes_filter/
├── include/
│   └── bayes_filter.hpp
├── src/
│   └── bayes_filter.cpp
├── launch/
│   └── bayes_filter.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🏁 Future Work

* Particle Filter version
* 3D mapping
* Multi-source plume simulation
* Real-world deployment

---

## ✨ Author

Akhil Sai — Robotics, ROS2, Probabilistic Robotics, C++
