#!/usr/bin/env python3
"""Generate SchurVINS calibration YAML files from calibration.json.

Reads the repo-agnostic calibration.json written by export_from_recon_debug.py
and writes SchurVINS-specific calibration YAMLs into svo_ros/param/calib/ce/,
mirroring the dataset folder structure.

Example (single dataset):
    python3 py/generate_ce_config.py \\
        --dataset-dir /host/mnt/nas3/user/ajones/datasets/VINS-Fusion/agx/20251031_SF/raw/251106-212953

Example (batch via scenario JSON):
    python3 py/generate_ce_config.py \\
        --dataset-root /host/mnt/nas3/user/ajones/datasets/VINS-Fusion \\
        --scenario-json /src/data_json/misc/odometry_rgb_ground_eval_small.json
"""
import argparse
import json
import pathlib

import numpy as np
import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
CONFIG_ROOT = REPO_ROOT / "svo_ros" / "param" / "calib" / "ce"

# IMU noise values from vio2/common/sensors_config.hpp (continuous-time densities).
IMU_NOISE = {
    "sigma_omega_c": 0.0001,        # gyro_sigma_n: rad/s / sqrt(Hz)
    "sigma_acc_c": 0.002,           # accel_sigma_n: m/s^2 / sqrt(Hz)
    "sigma_omega_bias_c": 0.00001,  # gyro_rws: rad/s * sqrt(Hz)
    "sigma_acc_bias_c": 0.0002,     # accel_rws: m/s^2 * sqrt(Hz)
}


def _format_camera_block(label: str, cal: dict, T_B_C: np.ndarray) -> str:
    """Format a single camera entry matching the EuRoC euroc_stereo.yaml style."""
    intrinsics = f"[{cal['fx']}, {cal['fy']}, {cal['cx']}, {cal['cy']}]"

    flat = T_B_C.flatten()
    t_rows = []
    for i in range(4):
        vals = ", ".join(str(v) for v in flat[i * 4:(i + 1) * 4])
        if i == 0:
            t_rows.append(f"    data: [{vals},")
        elif i == 3:
            t_rows.append(f"           {vals}]")
        else:
            t_rows.append(f"           {vals},")
    t_data = "\n".join(t_rows)

    return f"""\
- camera:
    label: {label}
    id: 0
    line-delay-nanoseconds: 0
    image_height: {cal['image_height']}
    image_width: {cal['image_width']}
    type: pinhole
    intrinsics:
      cols: 1
      rows: 4
      data: {intrinsics}
    distortion:
      type: radial-tangential
      parameters:
        cols: 1
        rows: 4
        data: [0, 0, 0, 0]
  T_B_C:
    cols: 4
    rows: 4
{t_data}"""


def write_schurvins_config(
    cal: dict,
    config_dir: pathlib.Path,
    overwrite: bool,
) -> pathlib.Path:
    """Write a SchurVINS stereo calibration YAML."""
    config_dir.mkdir(parents=True, exist_ok=True)
    out_path = config_dir / "ce_stereo.yaml"

    if out_path.exists() and not overwrite:
        print(f"  Skipping {out_path.name} (exists, use --overwrite)")
        return out_path

    T_B_C0 = np.array(cal["imu_from_cam0_rect"])
    T_B_C1 = np.array(cal["imu_from_cam1_rect"])
    imu_rate = cal.get("imu_rate", 800)

    # Camera section: formatted text to match EuRoC style exactly
    cameras_text = (
        f'label: "CompoundEye"\n'
        f"id: 0\n"
        f"cameras:\n"
        f"{_format_camera_block('cam0', cal, T_B_C0)}\n"
        f"{_format_camera_block('cam1', cal, T_B_C1)}\n"
    )

    # imu_params and imu_initialization: use yaml.dump()
    imu_params = {
        "imu_params": {
            "delay_imu_cam": 0.0,
            "max_imu_delta_t": 0.01,
            "acc_max": 176.0,
            "omega_max": 17,
            **IMU_NOISE,
            "sigma_integration": 0.0,
            "g": 9.81007,
            "imu_rate": imu_rate,
        },
        "imu_initialization": {
            "velocity": [0.0, 0.0, 0.0],
            "omega_bias": [0.0, 0.0, 0.0],
            "acc_bias": [0.0, 0.0, 0.0],
            "velocity_sigma": 2.0,
            "omega_bias_sigma": 0.01,
            "acc_bias_sigma": 0.1,
        },
    }
    # Represent lists inline [a, b, c] but dicts in block style
    class _Dumper(yaml.SafeDumper):
        pass

    _Dumper.add_representer(
        list,
        lambda dumper, data: dumper.represent_sequence(
            "tag:yaml.org,2002:seq", data, flow_style=True
        ),
    )
    imu_text = yaml.dump(imu_params, Dumper=_Dumper, default_flow_style=False,
                         sort_keys=False)

    out_path.write_text(cameras_text + "\n" + imu_text)
    print(f"  Wrote {out_path}")
    return out_path


def _dataset_rel_path(dataset_dir: pathlib.Path) -> str:
    """Extract relative dataset path by splitting on 'datasets/'."""
    dataset_dir_str = str(dataset_dir)
    if "datasets/" in dataset_dir_str:
        return dataset_dir_str.split("datasets/", 1)[1]
    raise RuntimeError(
        f"Dataset dir {dataset_dir} does not contain 'datasets/' in its path. "
        f"Cannot derive config directory structure."
    )


def _raw_root_rel_path(raw_root: str) -> str:
    """Extract relative path from raw_root by splitting on 'datasets/'."""
    if "datasets/" in raw_root:
        return raw_root.split("datasets/", 1)[1]
    raise RuntimeError(
        f"raw_root {raw_root} does not contain 'datasets/' in its path."
    )


def generate_for_dataset(dataset_dir: pathlib.Path, config_dir: pathlib.Path,
                         overwrite: bool):
    """Generate SchurVINS config for a single dataset."""
    cal_path = dataset_dir / "calibration.json"
    if not cal_path.exists():
        print(f"Warning: {cal_path} not found, skipping")
        return

    with open(cal_path) as f:
        cal = json.load(f)

    print(f"Dataset {dataset_dir.name} -> {config_dir}")
    write_schurvins_config(cal, config_dir, overwrite)


def main():
    parser = argparse.ArgumentParser(
        description="Generate SchurVINS config files from calibration.json."
    )
    parser.add_argument(
        "--dataset-dir", type=pathlib.Path,
        help="Single exported dataset directory containing calibration.json.",
    )
    parser.add_argument(
        "--dataset-root", type=pathlib.Path,
        help="Root of exported datasets (for batch mode with --scenario-json).",
    )
    parser.add_argument(
        "--scenario-json", type=pathlib.Path,
        help="Scenario JSON for batch mode.",
    )
    parser.add_argument(
        "--overwrite", action="store_true",
        help="Overwrite existing config files.",
    )
    args = parser.parse_args()

    print(f"Config output root: {CONFIG_ROOT}")

    if args.scenario_json:
        if not args.dataset_root:
            parser.error("--dataset-root required with --scenario-json")
        with open(args.scenario_json) as f:
            scenario = json.load(f)
        for group in scenario.get("dataset_groups", []):
            raw_root = group["raw_root"]
            rel_path = _raw_root_rel_path(raw_root)
            for ds in group.get("datasets", []):
                dataset_dir = args.dataset_root / rel_path / ds["id"]
                config_dir = CONFIG_ROOT / rel_path / ds["id"]
                generate_for_dataset(dataset_dir, config_dir, args.overwrite)
    elif args.dataset_dir:
        rel_path = _dataset_rel_path(args.dataset_dir)
        config_dir = CONFIG_ROOT / rel_path
        generate_for_dataset(args.dataset_dir, config_dir, args.overwrite)
    else:
        parser.error("Provide either --dataset-dir or --scenario-json")


if __name__ == "__main__":
    main()
