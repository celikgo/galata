#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise a staged onboard runner over the real POSIX serial adapter."""

import hashlib
import json
import errno
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import tempfile
import time
import unittest
import zlib


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
CONTROLLER = Path(os.environ.get("GALATA_TEST_CONTROLLER", "")).resolve()


def frame(sequence, timestamp_s, value):
    packet = struct.pack("<4sBBH Q d d", b"GLHW", 1, 0, 1, sequence, timestamp_s, value)
    return packet + struct.pack("<I", zlib.crc32(packet) & 0xFFFFFFFF)


def read_exact(descriptor, length, timeout_s=5.0):
    result = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(result) < length:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise AssertionError(f"timed out after {len(result)} of {length} bytes")
        readable, _, _ = select.select([descriptor], [], [], remaining)
        if not readable:
            raise AssertionError(f"timed out after {len(result)} of {length} bytes")
        result.extend(os.read(descriptor, length - len(result)))
    return bytes(result)


@unittest.skipUnless(
    os.name == "posix" and CLI.is_file() and CONTROLLER.is_file(),
    "requires the POSIX project CLI and test controller plugin",
)
class OnboardRunnerCLI(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-onboard-runner-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.model = self.root / "model.bin"
        self.model.write_bytes(b"controlled-model-v1\n")
        self.model_sha = hashlib.sha256(self.model.read_bytes()).hexdigest()
        self.controller_sha = hashlib.sha256(CONTROLLER.read_bytes()).hexdigest()
        self.master, slave = pty.openpty()
        self.addCleanup(os.close, self.master)
        self.endpoint = os.ttyname(slave)
        os.close(slave)
        self.manifest = self.root / "onboard.manifest"
        self.manifest.write_text(
            "format=galata-onboard-interface-manifest-v1\n"
            "qualification_state=not_qualified\n"
            "contains_executable=false\n"
            "target_platform=posix-bench-runner\n"
            "target.hardware_id=bench-airframe-01\n"
            "target.flight_computer_id=posix-fcu-v1\n"
            "target.firmware_id=bench-firmware-build-001\n"
            "target.emergency_stop_id=bench-estop-chain-01\n"
            "model_description=controlled model\n"
            "controller_description=controlled controller\n"
            "failsafe_action=disarm on link loss\n"
            "runtime.max_controller_time_s=0.005\n"
            "interface.id=serial-plugin-test-v1\n"
            "interface.sample_period_s=0.01\n"
            "interface.external_arming_required=true\n"
            "hardware.profile_id=serial-plugin-test-profile-v1\n"
            "hardware.transport=serial\n"
            f"hardware.endpoint={self.endpoint}|115200\n"
            "hardware.receive_timeout_ms=1000\n"
            "hardware.transmit_timeout_ms=1000\n"
            # Hosted CI PTYs can add scheduling latency before the first
            # complete frame reaches the runner. Keep the watchdog bounded,
            # while leaving enough margin for the test harness itself.
            "hardware.watchdog_timeout_s=1.0\n"
            "hardware.emergency_stop_required=true\n"
            "channel.sensor.0.name=airspeed_m_s\n"
            "channel.sensor.0.unit=m/s\n"
            "channel.sensor.0.frame=body\n"
            "channel.actuator.0.name=elevator_rad\n"
            "channel.actuator.0.unit=rad\n"
            "channel.actuator.0.frame=body\n"
            f"artifact.0.role=controller\nartifact.0.sha256={self.controller_sha}\n"
            f"artifact.1.role=model\nartifact.1.sha256={self.model_sha}\n",
            encoding="utf-8",
        )

    def invoke_runner(self, runtime, model, controller):
        process = subprocess.Popen(
            [
                str(runtime),
                "onboard",
                "run",
                self.manifest,
                "--model",
                model,
                "--controller",
                controller,
                "--operator",
                "runner-test",
                "--confirm",
                "ARM",
                "--cycles",
                "1",
                "--linger-ms",
                "500",
            ],
            cwd=self.root,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        # The child opens and configures the slave before it can receive. The
        # short wait also ensures its tcflush cannot discard the test packet.
        packet = frame(3, 0.0, 31.5)
        deadline = time.monotonic() + 5.0
        while True:
            if process.poll() is not None:
                stdout, stderr = process.communicate(timeout=5)
                self.fail(f"runner exited before opening the serial link: {stdout}{stderr}")
            if time.monotonic() >= deadline:
                stdout, stderr = process.communicate(timeout=5)
                self.fail(f"runner did not open the serial link: {stdout}{stderr}")
            try:
                # Opening the slave is not the same as completing its
                # tcflush/raw-mode setup. Let that transition settle before
                # publishing the first frame.
                time.sleep(0.1)
                os.write(self.master, packet)
                break
            except OSError as error:
                if error.errno != errno.EIO:
                    stdout, stderr = process.communicate(timeout=5)
                    self.fail(f"runner serial write failed ({error}): {stdout}{stderr}")
                time.sleep(0.01)
        output = bytearray()
        deadline = time.monotonic() + 5.0
        retry_sent = False
        while len(output) < 36:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                stdout, stderr = process.communicate(timeout=5)
                self.fail(f"runner produced no complete actuator packet: {stdout}{stderr}")
            readable, _, _ = select.select([self.master], [], [], min(remaining, 0.05))
            if readable:
                output.extend(os.read(self.master, 36 - len(output)))
            else:
                # A PTY master can accept a write before the child has
                # completed raw-mode setup; retry the same bounded frame once.
                if not retry_sent:
                    try:
                        os.write(self.master, packet)
                        retry_sent = True
                    except OSError as error:
                        if error.errno != errno.EIO:
                            stdout, stderr = process.communicate(timeout=5)
                            self.fail(f"runner serial retry failed ({error}): {stdout}{stderr}")
        output_packet = bytes(output)
        stdout, stderr = process.communicate(timeout=5)
        self.assertEqual(process.returncode, 0, stdout + stderr)
        result = json.loads(stdout)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["completed_cycles"], 1)
        self.assertEqual(result["observed_cycles"], 1)
        self.assertGreaterEqual(result["observed_controller_worst_case_s"], 0.0)
        self.assertGreaterEqual(result["observed_cycle_worst_case_s"],
                                result["observed_controller_worst_case_s"])
        return output_packet

    def test_deploys_starts_and_verifies_the_staged_runner(self):
        deployed = self.root / "deployed"
        deploy = subprocess.run(
            [
                str(CLI),
                "onboard",
                "deploy",
                self.manifest,
                deployed,
                "--runtime",
                CLI,
                "--artifact",
                f"model={self.model}",
                "--artifact",
                f"controller={CONTROLLER}",
            ],
            cwd=self.root,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(deploy.returncode, 0, deploy.stdout + deploy.stderr)
        deployment = json.loads(deploy.stdout)
        self.assertTrue(deployment["contains_executable"])
        self.assertEqual(deployment["qualification_state"], "not_qualified")
        self.assertTrue((deployed / "bin" / "galata").stat().st_mode & 0o111)

        verified = subprocess.run(
            [str(CLI), "onboard", "verify-deployment", deployed],
            cwd=self.root,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(verified.returncode, 0, verified.stdout + verified.stderr)
        verification = json.loads(verified.stdout)
        self.assertEqual(verification["status"], "verified")
        self.assertTrue(verification["contains_executable"])
        self.assertEqual(verification["artifact_count"], 2)

        unexpected = deployed / "unexpected.txt"
        unexpected.write_text("must not be hidden\n", encoding="utf-8")
        rejected = subprocess.run(
            [str(CLI), "onboard", "verify-deployment", deployed],
            cwd=self.root,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertNotEqual(rejected.returncode, 0)
        unexpected.unlink()
        linked = deployed / "artifacts" / "linked-controller"
        linked.symlink_to(deployed / "artifacts" / "controller")
        rejected = subprocess.run(
            [str(CLI), "onboard", "verify-deployment", deployed],
            cwd=self.root,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertNotEqual(rejected.returncode, 0)
        linked.unlink()

        packet = self.invoke_runner(
            deployed / "bin" / "galata",
            deployed / "artifacts" / "model",
            deployed / "artifacts" / "controller",
        )
        self.assertEqual(packet[:4], b"GLHW")
        self.assertEqual(struct.unpack_from("<H", packet, 6)[0], 1)
        self.assertEqual(struct.unpack_from("<Q", packet, 8)[0], 3)
        self.assertEqual(struct.unpack_from("<d", packet, 16)[0], 0.0)
        self.assertAlmostEqual(struct.unpack_from("<d", packet, 24)[0], 0.0315)
        self.assertEqual(
            struct.unpack_from("<I", packet, 32)[0], zlib.crc32(packet[:32]) & 0xFFFFFFFF
        )

    def test_deployed_runner_accepts_a_first_udp_sensor_frame_on_fixed_local_port(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(server.close)
        server.bind(("127.0.0.1", 0))
        remote_port = server.getsockname()[1]

        local_probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        local_probe.bind(("127.0.0.1", 0))
        local_port = local_probe.getsockname()[1]
        local_probe.close()

        udp_manifest = self.root / "udp.manifest"
        udp_manifest.write_text(
            self.manifest.read_text(encoding="utf-8")
            .replace("hardware.transport=serial", "hardware.transport=udp")
            .replace(
                f"hardware.endpoint={self.endpoint}|115200",
                f"hardware.endpoint=127.0.0.1:{remote_port}|{local_port}",
            ),
            encoding="utf-8",
        )
        deployed = self.root / "udp-deployed"
        deploy = subprocess.run(
            [
                str(CLI),
                "onboard",
                "deploy",
                udp_manifest,
                deployed,
                "--runtime",
                CLI,
                "--artifact",
                f"model={self.model}",
                "--artifact",
                f"controller={CONTROLLER}",
            ],
            cwd=self.root,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(deploy.returncode, 0, deploy.stdout + deploy.stderr)

        process = subprocess.Popen(
            [
                str(deployed / "bin" / "galata"),
                "onboard",
                "run",
                udp_manifest,
                "--model",
                deployed / "artifacts" / "model",
                "--controller",
                deployed / "artifacts" / "controller",
                "--operator",
                "udp-runner-test",
                "--confirm",
                "ARM",
                "--cycles",
                "1",
                "--linger-ms",
                "100",
            ],
            cwd=self.root,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        packet = frame(3, 0.0, 31.5)
        output_packet = None
        deadline = time.monotonic() + 5.0
        server.settimeout(0.05)
        while time.monotonic() < deadline:
            if process.poll() is not None:
                stdout, stderr = process.communicate(timeout=5)
                self.fail(f"UDP runner exited before the first sensor frame: {stdout}{stderr}")
            server.sendto(packet, ("127.0.0.1", local_port))
            try:
                output_packet, _ = server.recvfrom(36)
                break
            except socket.timeout:
                continue
        if output_packet is None:
            process.kill()
            stdout, stderr = process.communicate(timeout=5)
            self.fail(f"UDP runner produced no actuator packet: {stdout}{stderr}")

        stdout, stderr = process.communicate(timeout=5)
        self.assertEqual(process.returncode, 0, stdout + stderr)
        result = json.loads(stdout)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["completed_cycles"], 1)
        self.assertEqual(output_packet[:4], b"GLHW")
        self.assertEqual(struct.unpack_from("<Q", output_packet, 8)[0], 3)
        self.assertAlmostEqual(struct.unpack_from("<d", output_packet, 24)[0], 0.0315)
        self.assertEqual(
            struct.unpack_from("<I", output_packet, 32)[0],
            zlib.crc32(output_packet[:32]) & 0xFFFFFFFF,
        )


if __name__ == "__main__":
    unittest.main()
