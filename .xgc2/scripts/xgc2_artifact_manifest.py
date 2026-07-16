#!/usr/bin/env python3
"""Create a deterministic release-train manifest for camera Debian artifacts."""

import argparse
import hashlib
import json
import pathlib
import subprocess


def deb_field(path, field):
    return subprocess.check_output(["dpkg-deb", "-f", str(path), field], text=True).strip()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_manifest(arguments):
    deb_dir = pathlib.Path(arguments.deb_dir)
    output_dir = pathlib.Path(arguments.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts = []
    for deb in sorted(deb_dir.glob("*.deb")):
        artifacts.append(
            {
                "filename": deb.name,
                "sha256": sha256(deb),
                "size_bytes": deb.stat().st_size,
                "package": deb_field(deb, "Package"),
                "version": deb_field(deb, "Version"),
                "architecture": deb_field(deb, "Architecture"),
            }
        )
    if not artifacts:
        raise SystemExit("no .deb artifacts found in {}".format(deb_dir))
    manifest = {
        "schema": "xgc2.artifact-manifest.v1",
        "product": arguments.product,
        "product_version": arguments.product_version,
        "distribution": arguments.distribution,
        "architecture": arguments.architecture,
        "source_sha": arguments.source_sha,
        "ci": {
            "run_id": arguments.ci_run_id,
            "workflow": arguments.ci_workflow,
            "workflow_ref": arguments.ci_workflow_ref,
        },
        "artifacts": artifacts,
    }
    destination = output_dir / "{}-{}-{}.json".format(
        arguments.product, arguments.distribution, arguments.architecture
    )
    destination.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build")
    for name in (
        "deb_dir", "output_dir", "product", "product_version", "distribution",
        "architecture", "source_sha", "ci_run_id", "ci_workflow", "ci_workflow_ref",
    ):
        build.add_argument("--" + name.replace("_", "-"), required=True)
    build.set_defaults(function=build_manifest)
    arguments = parser.parse_args()
    arguments.function(arguments)


if __name__ == "__main__":
    main()
