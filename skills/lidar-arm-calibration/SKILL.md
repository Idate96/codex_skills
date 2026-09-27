---
name: lidar-arm-calibration
description: "Check or recalibrate the Mole/Menzi M445 front-left LiDAR extrinsic against the arm kinematics. Registers raw LiDAR points on the arm against TF-posed URDF meshes over several static arm poses, then compares LiDAR-mount and joint-offset models. Use when the mapped cut floor disagrees with the design or the Dig3D kinematic edge, after a sensor remount or URDF change, or to validate LiDAR↔arm consistency before depth debugging."
---

# LiDAR ↔ Arm Calibration

The full operator procedure is `mole_utils/docs/LIDAR_ARM_CALIBRATION.md` in `moleworks_ros` (main ≥ `ac0f2594f`).
This skill is the agent runbook around it.

Run this **before** debugging the Dig3D depth QP whenever the elevation map shows the cut floor above or below
design. The QP constrains the *kinematic* edge (URDF + encoders), while the operator and Terra judge depth
from the *LiDAR* map. On 2026-09-27 a 1.2° / 7 cm LiDAR mount error read as a "high cutting line": the map
showed the floor 5–7 cm too high.

## Tools

- `ros2 run mole_dev_tools capture_lidar_arm_pose <name> --out-dir <dir>`
  - Read-only: it never publishes.
  - Accumulates ~5 s of raw `/mole/livox_lidar_publisher/lidar_front_left` in CABIN, plus link TFs, joints and
    `robot_description`.
  - Refuses to save if the arm moves or clouds are missing.
- `ros2 run mole_dev_tools fit_lidar_arm_calibration <dir> --urdf <dir>/robot_description.urdf [--joints …] [--models …]`
  - Takes ~7 min for 10 poses.
  - Prints per-pose residuals, a condition number, plausibility warnings and a xacro-ready origin.
- If `ros2 run` can't find them, the workspace predates `ac0f2594f`.
  - Do not pull into the live robot checkout; other sessions keep uncommitted work there.
  - Either build `mole_dev_tools` from main in an isolated workspace (`$ros-worktree`), or use the prototypes
    in `~/mcap/dig/lidar_arm_calib_20260927/`.
  - The prototypes are `capture_pose.py <name>`, which writes to its cwd, and `joint_fit.py`.
  - Both use the same npz format.

## Capture Loop (real robot, operator present)

1. Set up the machine:
   - Engine to 1600 RPM via `$set-engine-rpm`.
   - Check `/machine_status` interlocks.
   - Refresh the MPC terrain SDF: `/mole/mobile_manipulator_mpc_node/terrain_collision/recompute_sdf_after_map_update`.
2. Capture the current pose first. It needs no motion.
3. Command motion through the OCS2 arm MPC (see `$ocs2-arm-experiments`).
   - With Terra's launch up, `mole_arm_mpc_controller` is the one registered `/mole/actuator_commands` publisher.
   - `$robot-move-to-position` refuses while that publisher exists.
   - Activate it with `ros2 lifecycle set /mole/mole_arm_mpc_controller activate`. It holds the measured pose.
   - Then send each move:
     `ros2 run mole_ocs2_arm_controller mole_m4_send_cyl_goal.py --call-reset --dtheta-deg 0 --dr-m <dr> --dz-m <dz> [--dpitch-deg <d>]`.
4. Pick 8–10 static poses:
   - Vary reach (3.6–5.8 m), height, and bucket angle.
   - Include **low poses near working depth**. Errors do not transfer as a fixed offset between configurations,
     so don't extrapolate from high poses.
   - Keep theta near the cabin heading, so the bucket stays in the front-left LiDAR's view.
5. Stay clear of the terrain:
   - Check the terrain under the bucket footprint (elevation map) before each low pose, keeping ≥0.25 m clearance.
   - The bucket's lowest point depends on its angle: ~4 cm below the edge at −45°, ~30 cm at −9° (heel down).
6. Things that happen during motion:
   - **An open bucket pushes the contact point far out.** Curl it (`--dpitch-deg +`) to reach close radii.
   - **Unreachable targets** end at joint limits and time out. Latch a hold with a zero-delta goal
     (`--dr-m 0 --dz-m 0`).
   - **Stalls:** twice on 2026-09-27 a goal produced saturated MPC commands with no joint motion, cause unknown.
     If there's no progress within ~10 s, deactivate the controller and investigate; don't keep pushing.
7. Finish:
   - Put the bucket somewhere safe.
   - `ros2 lifecycle set /mole/mole_arm_mpc_controller deactivate`.
   - Engine back to 900 RPM.

## Reading The Result

- **Adopt `lidar`-only when it removes the residual uniformly across poses.** On 2026-09-27 it went 3.9 → 0.8 cm.
- **`joint`-only** needing large offsets (over ~1°, or tele over 2 cm) with little gain means the joints are not the cause.
- **`lidar+joint` over all arm joints is degenerate.**
  - The LiDAR (CABIN x≈0.82) and the J_BOOM pivot (x≈0.89) sit nearly on the same axis, so LiDAR pitch and
    boom/stick offsets trade off.
  - The combined fit converges to arbitrary points with the same residual. Never adopt it.
- **Bucket pitch/roll offsets around 2°** are not identifiable by mesh registration (the sign flipped between
  fits). Resolve them with a physical edge-touch check.
- **Independent sanity check:** after the correction, ground-vs-wheel-bottom gaps at the two front wheels should
  become left/right consistent. On 2026-09-27 the mismatch went from 4.8 to 0.9 cm.

## Applying A Correction (user decision)

1. Update `livox_front_left_joint` in `mole.urdf.xacro`.
2. Handle its child frames:
   - Keep `Main_joint` (camera) unchanged, so the camera stays rigid with the LiDAR. Its extrinsic is
     LiDAR-relative and drives point-cloud colouring.
   - Re-derive `imu_box_base_joint` so CABIN→`imu_box_link` is unchanged and the estimator is unaffected.
3. Mirror the values into `calibration/camera_extrinsics.yaml` and the calibration README.
4. Run `pytest description/mole_description/test/test_camera_extrinsics.py`.
5. Check the whole chain:
   - `xacro` the model.
   - Compare CABIN poses of `livox_front_left`, `Main` and `imu_box_link` against the live `robot_description`.
6. Make it live:
   - Running nodes keep the old description until the low-level `robot_state_publisher` restarts or has its
     `robot_description` parameter updated.
   - Maps saved before the change stay biased; re-survey before judging depth.
7. The earlier `scripts/calc_lidar_icp_correction.py` fits roll/pitch only, from cabin-rotation consistency.
   - If cabin-swing mapping smears after a lateral correction, part of the lateral error may belong to the boom
     model instead.
