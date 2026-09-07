# SPDX-License-Identifier: Apache-2.0
"""A released dependency inventory must describe actual installed packages."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class DependencyNotices(unittest.TestCase):
    def fixture(self, scratch):
        build = scratch / "build"
        installed = build / "vcpkg_installed"
        (installed / "vcpkg").mkdir(parents=True)
        (build / "CMakeCache.txt").write_text(
            f"VCPKG_INSTALLED_DIR:PATH={installed}\nVCPKG_TARGET_TRIPLET:STRING=test-triplet\n",
            encoding="utf-8")
        status = []
        for name in ("eigen3", "yaml-cpp"):
            share = installed / "test-triplet/share" / name
            share.mkdir(parents=True)
            (share / "copyright").write_text(f"actual {name} notice\n", encoding="utf-8")
            status.append(f"Package: {name}\nVersion: 1.2.3\nPort-Version: 4\n"
                          "Architecture: test-triplet\nStatus: install ok installed")
        status.append("Package: removed\nVersion: 1\nArchitecture: test-triplet\n"
                      "Status: purge ok not-installed")
        (installed / "vcpkg/status").write_text("\n\n".join(status) + "\n", encoding="utf-8")
        return build

    def test_exact_notice_contents_versions_and_hashes_travel_together(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory)
            build = self.fixture(scratch)
            destination = scratch / "notices"
            result = subprocess.run([sys.executable, str(ROOT / "scripts/collect-dependency-notices.py"),
                                     str(build), str(destination)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            inventory = json.loads((destination / "dependencies.json").read_text(encoding="utf-8"))
            self.assertEqual([item["name"] for item in inventory], ["eigen3", "yaml-cpp"])
            for item in inventory:
                self.assertEqual(item["version"], "1.2.3")
                self.assertEqual(item["port_version"], "4")
                data = (destination / item["notice"]).read_bytes()
                self.assertEqual(data, f"actual {item['name']} notice\n".encode())
                self.assertEqual(item["notice_sha256"], hashlib.sha256(data).hexdigest())

    def test_missing_runtime_notice_prevents_packaging(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory)
            build = self.fixture(scratch)
            (build / "vcpkg_installed/test-triplet/share/eigen3/copyright").unlink()
            result = subprocess.run([sys.executable, str(ROOT / "scripts/collect-dependency-notices.py"),
                                     str(build), str(scratch / "notices")], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
