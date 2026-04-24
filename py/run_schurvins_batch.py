#!/usr/bin/env python3
"""Run SchurVINS on all datasets in a scenario JSON.

Iterates over datasets, starts roscore + SchurVINS for each, plays the rosbag,
converts imu_traj.txt to BodyPath.OdometryPose, and creates fake recon dirs
compatible with compare_vio_runs.py / eval_vio.py.

Run inside the schurvins container.
Prerequisites: pip install fastavro pyquaternion numpy scipy

Example:
    python3 py/run_schurvins_batch.py \\
        --scenario-json data_json/odometry_rgb_ground_eval_small.json \\
        --dataset-root /host/mnt/nas3/user/ajones/datasets/VINS-Fusion \\
        --save-dir /host/mnt/nas3/user/ajones/output/SchurVINS/runs/20260408 \\
        --recon-root /host/mnt/nas3/user/ajones/reconstructions/vins_fusion_comparison/20260402_rgb_ground_small_kStereo_save_debug_images/odometry_rgb_ground_eval_small/reconstructions
"""
import argparse
import json
import pathlib
import shutil
import subprocess
import time

import numpy as np
from ce_pose import Pose, convert_schurvins_traj
from scipy.spatial.transform import Rotation

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
CONFIG_ROOT = REPO_ROOT / "svo_ros" / "param" / "calib" / "ce"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run SchurVINS on all datasets in a scenario JSON."
    )
    parser.add_argument(
        "--scenario-json",
        type=pathlib.Path,
        required=True,
        help="Path to scenario JSON file",
    )
    parser.add_argument(
        "--dataset-root",
        type=pathlib.Path,
        required=True,
        help="Root of exported datasets (contains data.bag per dataset)",
    )
    parser.add_argument(
        "--save-dir",
        type=pathlib.Path,
        required=True,
        help="Root output dir (reconstructions/ subfolder created automatically)",
    )
    parser.add_argument(
        "--recon-root",
        type=pathlib.Path,
        required=True,
        help="Root of original VIO2 reconstruction dirs (for gps_samples)",
    )
    parser.add_argument(
        "--config-name",
        default="ce_stereo.yaml",
        help="Config filename to use (default: ce_stereo.yaml)",
    )
    parser.add_argument(
        "--playback-rate",
        type=float,
        default=1.0,
        help="Rosbag playback rate (default: 1.0)",
    )
    return parser.parse_args()


def datasets_from_scenario_json(json_path: pathlib.Path):
    """Yield (csi_dir, dataset_id, rel_path) for each dataset in the scenario JSON."""
    with open(json_path) as f:
        data = json.load(f)
    for group in data["dataset_groups"]:
        raw_root = group["raw_root"]
        if "datasets/" not in raw_root:
            print(
                f"WARNING: raw_root has no 'datasets/' in path: {raw_root}, skipping group"
            )
            continue
        rel_path = raw_root.split("datasets/", 1)[1]
        for dataset in group["datasets"]:
            dataset_id = dataset["id"]
            csi_dir = f"{raw_root}/{dataset_id}"
            yield csi_dir, dataset_id, rel_path


def kill_proc(proc, name):
    """Kill a subprocess and wait for it to exit."""
    if proc is None or proc.poll() is not None:
        return
    print(f"  Stopping {name} (pid {proc.pid})...")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        print(f"  Force killing {name}...")
        proc.kill()
        proc.wait()


def read_imu_from_cam_rect(calibration_json_path: pathlib.Path) -> Pose:
    """Read imu_from_cam0_rect from calibration.json and return as a Pose."""
    with open(calibration_json_path) as f:
        cal = json.load(f)
    T = np.array(cal["imu_from_cam0_rect"])
    q_xyzw = Rotation.from_matrix(T[:3, :3]).as_quat()  # scipy: [x, y, z, w]
    q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
    return Pose(q=q_wxyz, t=T[:3, 3].tolist())


def write_fake_reconstruction_log(save_dir: pathlib.Path, csi_dir: str):
    """Write a minimal reconstruction.log that eval tools can parse."""
    log_path = save_dir / "reconstruction.log"
    log_path.write_text(
        f"reconstruction_batch --csi-dir={csi_dir}\n" f"exit status: 0\n"
    )


def run_dataset(
    bag_path: pathlib.Path,
    calib_path: pathlib.Path,
    calibration_json_path: pathlib.Path,
    run_save_dir: pathlib.Path,
    playback_rate: float,
    csi_dir: str,
    recon_dir: pathlib.Path,
):
    """Run SchurVINS on a single dataset and convert output."""
    run_save_dir.mkdir(parents=True, exist_ok=True)

    # Create trace/logs dirs that SchurVINS expects
    trace_dir = run_save_dir / "trace"
    trace_dir.mkdir(exist_ok=True)
    logs_dir = run_save_dir / "logs"
    logs_dir.mkdir(exist_ok=True)

    # Copy calib file to run dir for reference
    shutil.copy2(calib_path, run_save_dir / "config.yaml")

    roscore_proc = None
    svo_proc = None

    try:
        # Start roscore
        roscore_log = open(run_save_dir / "roscore.log", "w")
        roscore_proc = subprocess.Popen(
            ["roscore"], stdout=roscore_log, stderr=subprocess.STDOUT
        )
        time.sleep(2)

        # Start SchurVINS with trace_dir pointing to our run directory
        svo_log = open(run_save_dir / "roslaunch.log", "w")
        svo_proc = subprocess.Popen(
            [
                "roslaunch",
                "svo_ros",
                "euroc_vio_stereo.launch",
                f"calib_file:={calib_path}",
            ],
            stdout=svo_log,
            stderr=subprocess.STDOUT,
            env={
                **dict(__import__("os").environ),
                # Override trace_dir so output goes to our run directory
            },
        )
        time.sleep(3)

        # Play rosbag
        print("  Playing rosbag...")
        subprocess.run(
            ["rosbag", "play", str(bag_path), "--rate", str(playback_rate)],
        )

        # Wait for SchurVINS to finish processing remaining data
        print("  Waiting for SchurVINS to finish processing...")
        time.sleep(5)

    finally:
        kill_proc(svo_proc, "svo_node")
        kill_proc(roscore_proc, "roscore")
        roscore_log.close()
        svo_log.close()

    # SchurVINS writes imu_traj.txt to its logs dir (relative to trace_dir).
    # Default trace_dir is <SchurVINS>/svo/trace, logs is ../../logs = <SchurVINS>/logs
    schurvins_logs = REPO_ROOT / "logs"
    traj_file = schurvins_logs / "imu_traj.txt"

    if not traj_file.exists():
        print(f"  WARNING: imu_traj.txt not found at {traj_file}")
        return False

    # Copy trajectory to run dir
    shutil.copy2(traj_file, run_save_dir / "imu_traj.txt")
    n_lines = sum(1 for _ in open(traj_file))
    print(f"  imu_traj.txt: {n_lines} lines")

    # Convert to BodyPath.OdometryPose
    print("  Converting imu_traj.txt to BodyPath.OdometryPose...")
    imu_from_cam_rect = read_imu_from_cam_rect(calibration_json_path)
    output_avro = run_save_dir / "BodyPath.OdometryPose"
    n_poses = convert_schurvins_traj(
        run_save_dir / "imu_traj.txt", output_avro, imu_from_cam_rect
    )
    print(f"  Wrote {n_poses} poses to {output_avro}")

    # Create fake reconstruction.log
    write_fake_reconstruction_log(run_save_dir, csi_dir)

    # Copy gps_samples from original reconstruction for vio debug viewer
    gps_src = recon_dir / "gps_samples"
    if gps_src.exists():
        shutil.copy2(gps_src, run_save_dir / "gps_samples")
        print(f"  Copied gps_samples from {recon_dir.name}")
    else:
        print(f"  WARNING: gps_samples not found at {gps_src}")

    return True


def main():
    args = parse_args()

    recons_dir = args.save_dir / "reconstructions"
    recons_dir.mkdir(parents=True, exist_ok=True)

    print(f"Scenario JSON:  {args.scenario_json}")
    print(f"Dataset root:   {args.dataset_root}")
    print(f"Save dir:       {args.save_dir}")
    print(f"Reconstructions:{recons_dir}")
    print(f"Recon root:     {args.recon_root}")
    print(f"Config name:    {args.config_name}")
    print()

    # Copy scenario JSON for reference
    shutil.copy2(args.scenario_json, recons_dir / args.scenario_json.name)

    summary_reconstructions = []

    for csi_dir, dataset_id, rel_path in datasets_from_scenario_json(
        args.scenario_json
    ):
        bag_path = args.dataset_root / rel_path / dataset_id / "data.bag"
        config_path = CONFIG_ROOT / rel_path / dataset_id / args.config_name
        calibration_json = (
            args.dataset_root / rel_path / dataset_id / "calibration.json"
        )
        run_save_dir = recons_dir / dataset_id
        orig_recon_dir = args.recon_root / dataset_id

        if not bag_path.exists():
            print(f"SKIP {dataset_id}: bag not found at {bag_path}")
            print()
            continue
        if not config_path.exists():
            print(f"SKIP {dataset_id}: config not found at {config_path}")
            print()
            continue
        if not calibration_json.exists():
            print(
                f"SKIP {dataset_id}: calibration.json not found at {calibration_json}"
            )
            print()
            continue

        print(f"{'=' * 60}")
        print(f"Dataset: {dataset_id}")
        print(f"  bag:    {bag_path}")
        print(f"  config: {config_path}")
        print(f"  save:   {run_save_dir}")
        print(f"  recon:  {orig_recon_dir}")
        print(f"{'=' * 60}")

        success = run_dataset(
            bag_path,
            config_path,
            calibration_json,
            run_save_dir,
            args.playback_rate,
            csi_dir,
            orig_recon_dir,
        )

        if success:
            summary_reconstructions.append(
                {
                    "test_case": {"csi_dir": csi_dir},
                    "reconstruction_dir": str(run_save_dir),
                }
            )
        print()

    # Write summary.json
    summary_path = recons_dir / "summary.json"
    summary = {"reconstructions": summary_reconstructions}
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Wrote {summary_path} ({len(summary_reconstructions)} datasets)")
    print(f"All datasets processed. Results in {recons_dir}")


if __name__ == "__main__":
    main()
