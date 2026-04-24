# Comparing VIO2 vs SchurVINS on CE Data

How to run end-to-end pipeline to benchmark our in-house VIO2 against SchurVINS
on the same CSI datasets. The comparison is fair by construction: SchurVINS is
fed the exact pixels VIO2 processed, not a re-implemented image pipeline.

**Final artifacts:**
- HTML regression comparison dashboard (EPE, CDFs, bad-segment counts) at
  `<output-root>/index.html`.

Commands are run in one of two containers, labeled on every stage heading:

- **ce_build_env** — Compoundeye monorepo container
- **schurvins** — SchurVINS container

Data moves between containers through the shared host filesystem.

The original vio2 reconstruction root_dir and schurvins output root_dirs
are here:
- vio2: `/host/mnt/nas3/user/ajones/reconstructions/vins_fusion_comparison/20260402_rgb_ground_small_kStereo_save_debug_images/odometry_rgb_ground_eval_small`
- schurvins: `/host/mnt/nas3/user/ajones/output/SchurVins/20260402_rgb_ground_small_kStereo_save_debug_images_use_euroc_config`


---

## Pipeline at a glance

```
┌────────────────────────┐   ┌────────────────────────┐   ┌────────────────────────┐
│ 1. VIO2 recon          │──▶│ 2. Export repo-        │──▶│ 3. ROS bag             │
│    (+ debug imgs)      │   │    agnostic data       │   │    (reused from        │
│    ce_build_env        │   │    ce_build_env        │   │     VINS-Fusion)       │
└────────────────────────┘   └────────────────────────┘   └────────────────────────┘
           │                                                          │
           │ BodyPath.OdometryPose                                    ▼
           │ (VIO2 baseline)                              ┌────────────────────────┐
           │                                              │ 4. Generate            │
           │                                              │    SchurVINS configs   │
           │                                              │    schurvins           │
           │                                              └────────────────────────┘
           │                                                          │
           │                                                          ▼
           │                                              ┌────────────────────────┐
           │                                              │ 5. Run SchurVINS       │
           │                                              │    + convert           │
           │                                              │    schurvins           │
           │                                              └────────────────────────┘
           │                                                          │
           ▼                                                          │
┌──────────────────────────────────────────────┐                      │
│ 6. Regression dashboard (compare_vio_runs.py)│◀─────────────────────┘
│    ce_build_env                              │
└──────────────────────────────────────────────┘
```

---

## Prerequisites

- CE monorepo checked out and buildable. `reconstruction_batch` is the main
  binary; build once via `env/build/build.sh -- reconstruction_batch`.
- SchurVINS Docker image built. Run `docker build -f docker/Dockerfile .` to
  Docker image.
- Change `SCHURVINS_HOST_PATH` in `docker/entrypoint.sh` to the path you cloned
  SchurVINS repo on host. When creating docker container, make sure path to SchurVINS
  repo in container is same as path on host and symlink it to `/catkin_ws/src/SchurVINS`.
- ROS bags already produced by the VINS-Fusion pipeline (same topics,
  interchangeable). See the Compound Eye VINS-Fusion fork —
  https://github.com/compound-eye/VINS-Fusion — specifically
  `compare_vio2_to_vins_fusion_instructions.md` Stage 3 for how the exported datasets
  and rosbags were created.
- Scenario JSON listing the datasets used to benchmark at
  `data_json/odometry_rgb_ground_eval_small.json`. You do not have to use
  the same dataset split. This is simply the one that was used for the original
  comparison.

---

## Stage 1 — VIO2 reconstructions with debug images (ce_build_env)

Make sure you've built reconstruction_batch:
```bash
./env/build/build.sh reconstruction_batch
```

Then run reconstruction using `scripts/analysis/reconstruction_test_runner.py`
with the `-Sdebug_img_interval=1` flag. This flag ensures all images
after being run through vio2's image processing pipeline are saved, so we can
feed the same images to any open-source method.

For which dataset split to run reconstruction on, the one used originally
is at `data_json/odometry_rgb_ground_eval_small.json`.

### Outputs consumed downstream (per dataset, under `--recon-output`)

```
<dataset_id>/
├── BodyPath.OdometryPose           ← VIO2 baseline trajectory (Stage 5, 6)
├── reconstruction.log
├── calibration/
│   ├── left_cam_imu_offset.lua     ← imu0_from_cam0_rect (Stage 2)
│   └── rectified_calibration0.lua  ← rectified intrinsics + baseline (Stage 2)
├── debug/
│   ├── _rgb.left/000000.png …      ← Stage 2 input
│   └── _rgb.right/000000.png …     ← stereo only
├── gps_samples                     ← copied into SchurVINS run dir (Stage 5)
└── …
summary.json                         ← consumed by compare_vio_runs.py (Stage 6)
```

---

## Stage 2 — Export repo-agnostic dataset (ce_build_env)

Reads `debug/_rgb.{left,right}/` + `calibration/` + the raw dataset's
`CsiFrameInfo` (for IMU) and produces a dataset any VIO repo can consume
without CE monorepo dependencies. Also writes a `calibration.json` per
dataset that the SchurVINS config generator (Stage 4) reads directly — no
Lua parsing or monorepo deps inside the SchurVINS container.

**Script:** `/src/py/dataset/export_repo_agnostic_dataset_from_recon.py`

The original exported datasets can be found here:
`/host/mnt/nas3/user/ajones/datasets/csi_datasets_for_opensource_repos`

They share the same subfolder structure as their original csi dataset
directories.

```
<scenario>/raw/<dataset_id>/
├── images/
│   ├── cam0/000000.png …
│   └── cam1/000000.png …      (stereo only)
├── frame_timestamps.csv       columns: frame_id, timestamp_s
├── imu_data.csv               columns: timestamp_s, acc_xyz, gyro_xyz
├── calibration.json           repo-agnostic intrinsics + extrinsics (Stage 4)
└── data.bag                   (produced by the VINS-Fusion pipeline; Stage 3)
```

### Single-dataset invocation (full export)

```bash
python3 /src/py/dataset/export_repo_agnostic_dataset_from_recon.py \
    --dataset-dir <path/to/csi_dataset_dir> \
    --recon-dir   <path/to/recon_dir> \
    --output-dir  <path/to/output_dir>
```

### Why this stage exists

CE's image pipeline uses a non-standard Division polynomial distortion model
plus dataset-specific rectification parameters (`rectified_height` from
`PairSettings0.lua`, `image_dim_scale` from `reconstruction_settings.lua`).
Re-implementing it in Python risks subtle pixel drift. Reusing reconstruction
debug output guarantees pixel-identical frames reach SchurVINS at zero extra
implementation cost.

### Calibration notes

- **`T_B_C` for cam0 = `imu0_from_cam0_rect`** — SchurVINS's "body" frame is
  the IMU, same convention as VINS-Fusion. This is NOT identity (VIO2's body
  is `cam0_rect`).
- **`T_B_C` for cam1 = `imu0_from_cam0_rect · [I | [baseline, 0, 0]ᵀ]`** —
  after stereo rectification, cam1 differs from cam0 only by a horizontal
  baseline.

---

## Stage 3 — ROS bags (reused from VINS-Fusion)

SchurVINS subscribes to the same topics as VINS-Fusion (`/cam0/image_raw`,
`/cam1/image_raw`, `/imu0`) with identical encodings (`mono8` images, standard
`sensor_msgs/Imu`). The bags produced by the VINS-Fusion pipeline are
fully reusable and were not recreated by this repo.

See the Compound Eye VINS-Fusion fork for how they were created:
https://github.com/compound-eye/VINS-Fusion — in particular
`compare_vio2_to_vins_fusion_instructions.md` Stage 3.

The bags live alongside each exported dataset as
`<dataset_dir>/data.bag`.

---

## Stage 4 — Generate SchurVINS YAML configs (schurvins)

Unlike VINS-Fusion (separate `cam*.yaml` + main config), SchurVINS uses a
single YAML with `cameras[]` + `imu_params` + `imu_initialization`. The
generator reads `calibration.json` from each exported dataset and writes one
config per dataset into `svo_ros/param/calib/ce/`, mirroring the dataset
folder structure.

**Script:** `py/generate_ce_config.py` (in the SchurVINS repo).

The config files will be generated under:
```
svo_ros/param/calib/ce
```
using the same subfolder structure as the original CSI dataset directories.

The config files for `data_json/odometry_rgb_ground_eval_small.json` are
already generated.

### Batch (all datasets in the scenario JSON)

```bash
python3 py/generate_ce_config.py \
    --dataset-root  <path/to/exported_datasets_root_dir> \
    --scenario-json data_json/odometry_rgb_ground_eval_small.json \
```

### Single dataset

```bash
python3 py/generate_ce_config.py \
    --dataset-dir <path/to/single/exported/csi_dir>
```

### IMU noise parameters

Values are taken directly from VIO2's `vio2/common/sensors_config.hpp` with
no conversion — all three systems (VIO2, VINS-Fusion, SchurVINS) use
continuous-time noise spectral densities, just with different YAML key names:

| VIO2 (`sensors_config.hpp`) | SchurVINS YAML key      | Value   |
|-----------------------------|-------------------------|---------|
| `accel_sigma_n`             | `sigma_acc_c`           | 0.002   |
| `gyro_sigma_n`              | `sigma_omega_c`         | 0.0001  |
| `accel_rws`                 | `sigma_acc_bias_c`      | 0.0002  |
| `gyro_rws`                  | `sigma_omega_bias_c`    | 0.00001 |

---

## Stage 5 — Run SchurVINS + convert to `BodyPath.OdometryPose` (schurvins)

**Script:** `py/run_schurvins_batch.py` (in the SchurVINS repo).

For each dataset in the scenario JSON, the script:

1. Starts `roscore`.
2. Starts `roslaunch svo_ros euroc_vio_stereo.launch calib_file:=<config>`
   with the per-dataset YAML produced in Stage 4. (It's called `euroc_vio_stereo.launch`,
   but the launch file is actually agnostic for any stereo vio dataset.)
3. Plays `data.bag` in the corresponding dataset folder.
4. Copies `logs/imu_traj.txt` (SchurVINS's raw output) into the run save dir.
5. Converts `imu_traj.txt` → `BodyPath.OdometryPose` (Avro) via the
   self-contained `ce_pose.py` — no CE monorepo dependencies required inside
   the SchurVINS container.
6. Copies `gps_samples` from the matching VIO2 reconstruction (via
   `--recon-root`) (helpful for visualizing trajectories in vio debug tool).
7. Writes a minimal `reconstruction.log` so `eval_vio.py` accepts the run.
8. Writes `summary.json` listing successful datasets.

### Batch invocation

```bash
python3 py/run_schurvins_batch.py \
    --scenario-json data_json/odometry_rgb_ground_eval_small.json \
    --dataset-root  <path/to/exported_datasets_root_dir> \
    --recon-root    <path/to/vio2/reconstructions_root_dir> \
    --save-dir      <path/to/save/dir>
```

`--recon-root` points at the per-dataset VIO2 reconstruction directories from
Stage 1; only `gps_samples` is read from them. `--dataset-root` resolves each
`<scenario>/raw/<dataset_id>` relative path against the exported-datasets
tree, mirroring the CE CSI dataset layout.

### Coordinate transform (summarized)

SchurVINS outputs `svo_world_from_imu` to `imu_traj.txt` in a Z-up,
gravity-aligned world frame — same convention as VINS-Fusion.
`BodyPath.OdometryPose` expects `vio2_world_from_cam0_rect` in VIO2 world
frame (X-right, Y-down, Z-forward). `ce_pose.py` applies:

```
vio2_world_from_cam0_rect = VIO2WORLD_FROM_SVOWORLD
                             · svo_world_from_imu
                             · imu_from_cam0_rect
```

where `VIO2WORLD_FROM_SVOWORLD` is +90° about X (identical to the
`VIO2WORLD_FROM_VINSWORLD` transform used in the VINS-Fusion pipeline).

---

## Stage 6 — Comparing VIO2 to SchurVINS via Regression Dashboard (ce_build_env)

**Script:** `/src/py/vio/compare_vio_runs.py`. Runs EPE + bad-segment +
CDF evaluation on each labeled run, then emits a single comparison dashboard.

The root directories passed for vio2 and schurvins must contain the
`reconstructions` subdirectory, which contains the outputs for each dataset.

```bash
./env/build/run.sh python3 py/vio/compare_vio_runs.py \
    --runs \
        "vio2:<path/to/vio2/recon_root_dir>" \
        "schurvins:<path/to/schurvins/output_root_dir>" \
    --output-root <path/to/save/output>
```

Open `<output-root>/index.html` to view side-by-side metrics.

---
