# dwa_local_planner (ROS 2 Humble)

A minimal **custom DWA** (Dynamic Window Approach) local planner designed to work with **TurtleBot3** in **Gazebo** without Nav2/dwb. It samples velocity commands within the dynamic window, simulates short trajectories, scores them, and publishes the best `cmd_vel`. Also publishes an RViz `Marker` for the chosen trajectory.

> Tested with Ubuntu 22.04 + ROS 2 Humble + turtlebot3_gazebo.

---

## 0) Quick Features
- Subscribes: `/odom`, `/scan`
- Publishes: `/cmd_vel`, `dwa/best_trajectory` (Marker, `base_link` frame)
- Set goal:
  - via params `goal_x`, `goal_y`, or
  - publish a `PoseStamped` to `/goal_pose`
- Tunable params: velocity/acc limits, simulation window, cost weights.
- Collision check against LaserScan (robot frame).

---
desktop/
   custom_dwa_planner/
      ├─ CMakeLists.txt

         ├─ package.xml
            
            ├─ README.md
               
               ├─ launch/
         
                  │  └custom_dwa_planner.launch.py
                      ├─ src/
                  │  └─ custom_dwa_planner_node.py
                      └─ config/
                      └─ params.yaml


## 1) Install & Build

```bash
# If you don't already have a workspace:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src


# Install TurtleBot3 sim pkgs if needed
sudo apt update
sudo apt install -y ros-humble-turtlebot3*

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## 2) Run with TurtleBot3 in Gazebo

Terminal A – launch Gazebo world:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Insert a few boxes via Gazebo's **Insert** tool to create obstacles.

Terminal B – start the planner:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch dwa_local_planner dwa_planner.launch.py
```

Terminal C – set a goal (odom frame coordinates):
```bash
# Example goal at (x=1.5, y=0.0). You can change at runtime.
ros2 param set /dwa_planner goal_x 1.5
ros2 param set /dwa_planner goal_y 0.0

# Or publish a PoseStamped
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "header:
  frame_id: 'odom'
pose:
  position: {x: 1.5, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" -1
```

Optional: In RViz, add a **Marker** display for topic `/dwa/best_trajectory` (frame `base_link`) to see the selected local path.

---

## 3) How it Works (Short)

1. **Dynamic Window** around current velocity based on acceleration limits.
2. Sample `(v, ω)` pairs.
3. **Rollout** short trajectories using the unicycle model for `sim_time` with step `sim_dt` in the **robot frame**.
4. Check each predicted pose against the **LaserScan** ray at the corresponding angle. If the predicted robot body would intersect a scan return (within `robot_radius + safety_margin`), the trajectory is discarded.
5. Score the collision-free trajectories:
   - `heading_score`: inverse of distance from trajectory endpoint to goal (in robot frame).
   - `clearance_score`: normalized minimum clearance along the path.
   - `velocity_score`: prefer faster linear velocity.
   - `smoothness_score`: mild preference for lower |ω|.
6. Publish the **best** `(v, ω)` as `/cmd_vel` and output a `Marker` for RViz.

---

## 4) Parameters (config/params.yaml)
Key params you may want to tune:
- `goal_x`, `goal_y` – navigation target in `odom`
- `max_vel_x`, `max_vel_theta` – robot limits (TB3 burger defaults included)
- `acc_lim_x`, `acc_lim_theta` – used to form the dynamic window
- `sim_time`, `sim_dt` – prediction horizon
- `v_samples`, `omega_samples` – sampling density
- `robot_radius`, `safety_margin` – for collision checks
- `*_weight` – trade-offs for cost function

---

## 5) Expected Output
- TurtleBot3 should **move toward the goal** while **avoiding obstacles**.
- You will see meaningful `INFO` / `WARN` logs from the node (e.g., when no valid trajectory exists).
- A **short README**  with setup instructions is included.
- RViz Marker shows the selected local path in `base_link` frame.

---

## 6) Troubleshooting
- Robot not moving?
  - Confirm topics: `ros2 topic echo /scan` and `/odom` should stream data.
  - Check `TURTLEBOT3_MODEL=burger` in the Gazebo terminal.
  - Try reducing `v_samples`, `omega_samples` to keep CPU low.
- Spins in place or oscillates?
  - Increase `heading_weight` and `smoothness_weight`, reduce `velocity_weight`.
  - Increase `sim_time` a bit (e.g., 1.5).
- Colliding with obstacles?
  - Increase `robot_radius` or `safety_margin`.
  - Increase `clearance_weight`.
- Goal frame confusion:
  - The node expects goals in **odom**. If publishing from RViz using another frame, use TF or republish accordingly.

---

## 7) License
MIT
