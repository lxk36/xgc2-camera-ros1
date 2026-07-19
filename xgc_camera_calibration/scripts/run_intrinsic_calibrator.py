#!/usr/bin/env python3
"""Run ROS camera_calibration in a caller-owned result directory."""

import argparse
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path


def parser():
    result = argparse.ArgumentParser()
    result.add_argument("--image-topic", default="/usb_cam/image_raw")
    result.add_argument("--camera-namespace", default="/usb_cam")
    result.add_argument("--camera-name", default="usb_cam")
    result.add_argument("--board-size", default="11x8")
    result.add_argument("--square-size", type=float, default=0.03)
    result.add_argument(
        "--output-dir",
        default=os.path.expanduser("~/.local/state/xgc2/camera/calibrations/usb_cam/intrinsic"),
    )
    result.add_argument("--output-file", default="")
    result.add_argument("--pattern", choices=("chessboard", "circles", "acircles", "charuco"), default="chessboard")
    result.add_argument("--max-chessboard-speed", type=float)
    return result


def non_ros_arguments(arguments):
    """Return CLI arguments after removing ROS 1 remapping tokens."""
    return [argument for argument in arguments if ":=" not in argument]


def atomic_write(path, content):
    descriptor, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def collect_result(archive, output_dir, output_file):
    if not archive.is_file():
        print("No {} was generated; use CALIBRATE then SAVE before closing.".format(archive), file=sys.stderr)
        return False
    with tarfile.open(str(archive), "r:gz") as calibration_archive:
        candidates = [member for member in calibration_archive.getmembers() if Path(member.name).name == "ost.yaml"]
        if len(candidates) != 1 or not candidates[0].isfile():
            raise RuntimeError("calibration archive does not contain exactly one ost.yaml")
        stream = calibration_archive.extractfile(candidates[0])
        if stream is None:
            raise RuntimeError("could not read ost.yaml from calibration archive")
        yaml_bytes = stream.read()
    destination = Path(output_file).expanduser() if output_file else output_dir / "intrinsics.yaml"
    destination.parent.mkdir(parents=True, exist_ok=True)
    atomic_write(destination, yaml_bytes)
    shutil.copy2(str(archive), str(output_dir / "calibrationdata.tar.gz"))
    print("Saved intrinsic YAML to {}".format(destination), flush=True)
    print("Saved original calibration archive to {}".format(output_dir / "calibrationdata.tar.gz"), flush=True)
    return True


def main():
    # roslaunch appends tokens such as __name:=... and __log:=....  This
    # wrapper owns ordinary argparse flags, so remove ROS remappings first.
    args = parser().parse_args(non_ros_arguments(sys.argv[1:]))
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    archive = Path("/tmp/calibrationdata.tar.gz")
    try:
        archive.unlink()
    except FileNotFoundError:
        pass
    command = [
        "/opt/ros/noetic/bin/rosrun",
        "camera_calibration",
        "cameracalibrator.py",
        "--size",
        args.board_size,
        "--square",
        str(args.square_size),
        "--pattern",
        args.pattern,
        "--camera_name",
        args.camera_name,
        "--no-service-check",
    ]
    if args.max_chessboard_speed is not None:
        command.extend(["--max-chessboard-speed", str(args.max_chessboard_speed)])
    command.extend(["image:=" + args.image_topic, "camera:=" + args.camera_namespace])

    print("Calibration results will be collected below {}".format(output_dir), flush=True)
    print("After CALIBRATE, press SAVE in the camera_calibration window.", flush=True)
    return_code = 0
    try:
        return_code = subprocess.call(command, cwd=str(output_dir))
    except KeyboardInterrupt:
        return_code = 130
    try:
        collected = collect_result(archive, output_dir, args.output_file)
    except (OSError, RuntimeError, tarfile.TarError) as error:
        print("Could not collect intrinsic result: {}".format(error), file=sys.stderr)
        return 1
    if return_code == 0 and not collected:
        return 1
    return return_code


if __name__ == "__main__":
    sys.exit(main())
