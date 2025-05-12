# Sagittarius Maze‑Solver

Solve a printed maze with a **Sagittarius 6‑DoF robotic arm**.
(Code in ./src)
The workflow has two layers:

| layer | node | file | role |
|-------|------|------|------|
| low‑level executor | `/sgr_ctrl` (action server) | `src/sgr_ctrl.py` | wraps MoveIt!, plans poses, controls the two‑finger gripper |
| high‑level task   | `/txt_waypoint_executor` (action client) | `src/txt_waypoint_executor.py` | reads `point.txt`, streams goals to `/sgr_ctrl` |
| vision pre‑step   | *(stand‑alone)* | `maze_solver.py` | converts a binary maze image → ordered `point.txt` |

---

## Dependencies

| package | notes |
|---------|-------|
| ROS Noetic (or Melodic) desktop‑full | core ROS |
| MoveIt! (`ros‑noetic‑moveit`) | motion planning |
| `opencv‑python` `numpy` `scikit‑learn` | for `maze_solver.py` |
| `sgr_msgs` (custom) | message definitions |
| `sdk_sagittarius_arm` | servo‑current service |

```bash
pip install opencv-python numpy scikit-learn   # extra Python deps
```

## Prepare Maze

If you need to reproduce the process, you must first calibrate the coordinates of the four corners of the maze in the robotic arm's coordinate system.
This is necessary in order to derive the transformation function between OpenCV's pixel coordinates and the robotic arm's coordinate system.
(Otherwise, it will not be possible to obtain the correct coordinate points for the robotic arm's motion path.)

Based on your coordinate system design, you need to modify the coordinate transformation logic in maze_solver.py between lines 141 to 149.

Additionally, you need to prepare a high-quality maze image suitable for binarization processing.

Once you have the maze image, modify line 152 in maze_solver.py to set the correct image path.
Then, run the script using python maze_solver.py to compute the robotic arm's motion path.
The resulting path points will be saved in point.txt, and you need to move this file into the ./config directory.

## Robotic Arm Action

```bash

# Launch MoveIt!, sgr_ctrl server and waypoint client
roslaunch sgr_waypoint_runner waypoint_demo.launch \
          txt_path:=~/point.txt          \
          pick_pose:="0.00 -0.15 0.05"    \ (The position of the little block which will be picked, the z coordinate will control the height of the arm when moving.)
          arm_ns:=sgr532

grab at pick_pose → follow every point in point.txt → open gripper at last point → return to pick_pose with gripper open.