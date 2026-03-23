# 🤖 Python Basics for Robotics — Course Project

A hands-on robotics project built entirely in Python, where I programmed a TurtleBot3 (simulation) and a FastBot (real robot) to perform autonomous motion control, sensor reading, and basic obstacle avoidance — all from scratch.

---

## 📌 Overview

This project was the capstone of my *Python for Robotics* course. I wrote two versions of a robot control program — one procedural and one object-oriented — and implemented three robot algorithms on top of them. The programs were tested first in a Gazebo simulation and then deployed live on a real robot running in Barcelona, Spain, connected remotely.

---

## 🧱 Project Structure

```
ros2_ws/src/python_basics_project/python_basics_project/
├── robot_interface.py          # ROS2 interface layer (pre-provided, not modified)
├── robot_control_noclass.py    # My procedural robot control program
├── robot_control_classed.py    # My object-oriented robot control program
└── __init__.py
```

---

## ⚙️ What I Built

### Part 1 — Robot Familiarization
- Launched the TurtleBot3 simulation in **Gazebo** and visualized sensor data in **RViz**
- Remotely connected to a live **FastBot** robot and drove it using the teleop keyboard interface
- Compared what the robot perceives through its laser scanner vs. what it physically does

---

### Part 2 — Robot Control Programs (Procedural + OOP)

I wrote **30 functions** across two versions of the same program:

**Movement (14 functions)**
- Get/set linear and angular velocities
- Stop, move forward/backward, turn left/right
- Timed movement (move for N seconds)
- Distance-based movement using open-loop control: `distance = speed × time`
- Angle-based turning using open-loop control: `angle = angular_speed × time`

**Laser Scanner (13 functions)**
- Read min/max scan angles, angle increment, min/max detectable range
- Get full scan range array or range at a specific index
- Directly read front, back, left, right scan distances
- Find minimum and maximum finite (non-`inf`) range values with their indices

**Odometry (3 functions)**
- Get current X, Y, Z position as a dictionary
- Get current Roll, Pitch, Yaw orientation as a dictionary
- Calculate 2D Euclidean distance between two positions: `d = √((x2−x1)² + (y2−y1)²)`

The **OOP version** wraps all of these into a clean class structure, making it far easier to reuse and extend compared to the flat procedural version.

---

### Part 3 — Robot Algorithms (OOP only)

#### 🔍 Obstacle Prediction
- Extracted the **frontal 90°** of laser scan data (45° left and right of centre)
- Filtered out `inf` values and found the minimum range
- If above threshold (0.3 m) → `"none"`
- If below threshold → used Python's `statistics.multimode()` on the front ranges:
  - Multiple modes → `"obstacle"`
  - Single mode → `"wall"`

#### 🧭 Direction Tracking
- Read the robot's **yaw** value from odometry
- Divided the full ±π yaw range into **16 equal segments**
- Mapped each segment to a compass direction (N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW)
- Printed the current direction as a 3-character string on each update

#### 🚧 Naive Obstacle Avoider
- Divided the full 360° laser scan into **8 segments of 45° each**
- Used only: Left, Front-Left, Front, Front-Right, Right
- Logic:
  - Front clear → move forward
  - Front-left or front-right blocked → turn to opposite side
  - Front blocked → turn toward the side with more space
  - All frontal segments blocked → turn toward the side with most space
- Tracked **cumulative distance travelled** using Euclidean distance across iterations
- Ran for up to 5 minutes (300 seconds) before auto-stopping

---

## 🛠️ Tech Stack

| Tool | Purpose |
|------|---------|
| Python 3 | All programming |
| ROS 2 | Robot middleware |
| Gazebo | Physics simulation |
| RViz | Sensor visualization |
| TurtleBot3 | Simulated robot |
| FastBot | Real robot (remote lab, Barcelona) |

---

## 🚀 Running the Programs

**Launch simulation (separate terminal):**
```bash
source ~/simulation_ws/install/setup.bash
ros2 launch turtlebot3_gazebo main_turtlebot3_lab.launch.xml
```

**Run the procedural control program:**
```bash
cd ~/ros2_ws/src/python_basics_project/python_basics_project
python3 ./robot_control_noclass.py
```

**Run the OOP control program (with algorithms):**
```bash
cd ~/ros2_ws/src/python_basics_project/python_basics_project
python3 ./robot_control_classed.py
```

**Teleop (for manual override during testing):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 💡 Key Takeaways

- Writing the same program in both procedural and OOP styles made the advantages of OOP immediately obvious — reusability, readability, and scalability all improved dramatically.
- Open-loop control (using `speed × time`) is simple but imprecise; good enough for basic demos, but real-world accuracy needs closed-loop feedback.
- Testing in simulation first before touching the real robot is not just good practice — it's essential. Bugs that seem harmless in code can be quite chaotic on a physical robot.
