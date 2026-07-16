#!/usr/bin/env python3

import importlib.util
import io
import tarfile
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_intrinsic_calibrator.py"
SPEC = importlib.util.spec_from_file_location("run_intrinsic_calibrator", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class IntrinsicResultCollectionTest(unittest.TestCase):
    def test_extracts_ost_yaml_to_owned_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "calibrationdata.tar.gz"
            payload = b"camera_name: usb_cam\nimage_width: 640\nimage_height: 480\n"
            with tarfile.open(str(archive), "w:gz") as stream:
                info = tarfile.TarInfo("ost.yaml")
                info.size = len(payload)
                stream.addfile(info, io.BytesIO(payload))
            output = root / "runtime"
            output.mkdir()
            self.assertTrue(MODULE.collect_result(archive, output, ""))
            self.assertEqual((output / "intrinsics.yaml").read_bytes(), payload)
            self.assertTrue((output / "calibrationdata.tar.gz").is_file())

    def test_missing_archive_is_reported_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertFalse(MODULE.collect_result(root / "missing.tar.gz", root, ""))
            self.assertFalse((root / "intrinsics.yaml").exists())


if __name__ == "__main__":
    unittest.main()
