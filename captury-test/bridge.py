#!/usr/bin/env python3
"""
bridge.py — Python wrapper around bridge.exe.

Launches the C++ bridge as a subprocess, reads pose data from its stdout,
formats as OSC, and sends to TouchDesigner (or any OSC receiver).

Usage:
    python bridge.py --captury-host 192.168.1.50 \
                     --osc-host 127.0.0.1 \
                     --osc-port 7000

OSC address scheme:
    /captury/actor/<id>/joint/<jointName>   args: x, y, z, rx, ry, rz
    /captury/actor/<id>/timestamp           arg:  timestamp_us (int)
    /captury/actor/<id>/quality             arg:  trackingQuality (int)
    /captury/actor/<id>/joints              arg:  comma-separated joint names string
                                                  (sent once per actor)

Tweak SCALE / coordinate handling below if your downstream tool expects
meters or different axes. Captury reports translation in millimeters.
"""

import argparse
import os
import subprocess
import sys
import threading
from pathlib import Path

from pythonosc.udp_client import SimpleUDPClient
from pythonosc import osc_bundle_builder, osc_message_builder

# Captury translations are in millimeters. TouchDesigner / most tools want meters.
SCALE = 0.001

# If your downstream tool wants Y-up but Captury is Z-up (or vice versa), swap here.
# Captury is Y-up by default per the docs; leave as-is unless you see flipped data.
def transform_position(x, y, z):
    return (x * SCALE, y * SCALE, z * SCALE)


def transform_rotation(rx, ry, rz):
    # Euler angles in degrees per Captury. Pass through.
    return (rx, ry, rz)


class Bridge:
    def __init__(self, exe_path, captury_host, captury_port,
                 osc_host, osc_port, verbose=False):
        self.exe_path = Path(exe_path).resolve()
        self.captury_host = captury_host
        self.captury_port = captury_port
        self.osc = SimpleUDPClient(osc_host, osc_port)
        self.verbose = verbose
        self.proc = None

        # actor_id -> list of joint names (in order matching transforms)
        self.actor_joints = {}

    def run(self):
        if not self.exe_path.exists():
            sys.exit(f"bridge.exe not found at {self.exe_path}")

        cmd = [str(self.exe_path), self.captury_host, str(self.captury_port)]
        print(f"[py] launching: {' '.join(cmd)}", file=sys.stderr)

        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.PIPE,
            text=True,
            bufsize=1,  # line-buffered
        )

        # Pump stderr from the C++ process to our stderr in a background thread,
        # so the user sees its log messages.
        stderr_thread = threading.Thread(
            target=self._pump_stderr, daemon=True
        )
        stderr_thread.start()

        try:
            for line in self.proc.stdout:
                self._handle_line(line.rstrip("\n"))
        except KeyboardInterrupt:
            print("[py] interrupted, shutting down", file=sys.stderr)
        finally:
            self._shutdown()

    def _pump_stderr(self):
        for line in self.proc.stderr:
            sys.stderr.write(line)
            sys.stderr.flush()

    def _handle_line(self, line):
        if not line:
            return

        # Cheap dispatch on first token
        if line.startswith("POSE "):
            self._handle_pose(line)
        elif line.startswith("ACTOR "):
            self._handle_actor(line)
        elif line.startswith("ACTOR_CHANGED"):
            if self.verbose:
                print(f"[py] {line}", file=sys.stderr)
        elif line == "READY":
            print("[py] bridge ready, awaiting poses...", file=sys.stderr)
        else:
            # Unknown line; pass through for visibility
            if self.verbose:
                print(f"[py] (unknown) {line}", file=sys.stderr)

    def _handle_actor(self, line):
        # ACTOR <id> <numJoints> <name1> <name2> ...
        parts = line.split()
        try:
            actor_id = int(parts[1])
            num_joints = int(parts[2])
            names = parts[3:3 + num_joints]
        except (IndexError, ValueError):
            print(f"[py] malformed ACTOR line: {line}", file=sys.stderr)
            return

        self.actor_joints[actor_id] = names
        print(f"[py] actor {actor_id} announced with {len(names)} joints",
              file=sys.stderr)

        # Send the joint name list once so TD knows what's coming.
        self.osc.send_message(
            f"/captury/actor/{actor_id}/joints",
            ",".join(names)
        )

    def _handle_pose(self, line):
        # POSE <actorId> <ts> <numTransforms> <quality> <name1> <x> <y> <z> <rx> <ry> <rz> ...
        parts = line.split()
        try:
            actor_id = int(parts[1])
            timestamp = int(parts[2])
            num_transforms = int(parts[3])
            quality = int(parts[4])
        except (IndexError, ValueError):
            print(f"[py] malformed POSE line: {line[:100]}", file=sys.stderr)
            return

        # Each joint is 7 tokens: name + 6 floats
        joint_tokens = parts[5:]
        expected_len = num_transforms * 7
        if len(joint_tokens) < expected_len:
            # Truncated; just process what we have
            num_transforms = len(joint_tokens) // 7

        # Build a single OSC bundle per frame so TD receives it atomically.
        bundle = osc_bundle_builder.OscBundleBuilder(
            osc_bundle_builder.IMMEDIATELY
        )

        # Add timestamp + quality messages first
        msg = osc_message_builder.OscMessageBuilder(
            address=f"/captury/actor/{actor_id}/timestamp"
        )
        msg.add_arg(timestamp)
        bundle.add_content(msg.build())

        msg = osc_message_builder.OscMessageBuilder(
            address=f"/captury/actor/{actor_id}/quality"
        )
        msg.add_arg(quality)
        bundle.add_content(msg.build())

        # Add one message per joint
        for i in range(num_transforms):
            base = i * 7
            try:
                name = joint_tokens[base]
                x = float(joint_tokens[base + 1])
                y = float(joint_tokens[base + 2])
                z = float(joint_tokens[base + 3])
                rx = float(joint_tokens[base + 4])
                ry = float(joint_tokens[base + 5])
                rz = float(joint_tokens[base + 6])
            except (IndexError, ValueError):
                continue

            x, y, z = transform_position(x, y, z)
            rx, ry, rz = transform_rotation(rx, ry, rz)

            msg = osc_message_builder.OscMessageBuilder(
                address=f"/captury/actor/{actor_id}/joint/{name}"
            )
            for v in (x, y, z, rx, ry, rz):
                msg.add_arg(v)
            bundle.add_content(msg.build())

        self.osc.send(bundle.build())

    def snap(self, x: float = 0.0, z: float = 0.0, heading: float = 500.0):
        """Snap a skeleton at (x, z) mm with the given heading in degrees.
        heading > 360 means any orientation. Defaults to volume center (0, 0)."""
        if not self.proc or self.proc.poll() is not None:
            print("[py] snap: bridge not running", file=sys.stderr)
            return
        self.proc.stdin.write(f"SNAP {x} {z} {heading}\n")
        self.proc.stdin.flush()

    def _shutdown(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def main():
    ap = argparse.ArgumentParser(
        description="Captury Live → OSC bridge")
    ap.add_argument("--captury-host", required=True,
                    help="IP address of the Captury Live machine")
    ap.add_argument("--captury-port", type=int, default=2101,
                    help="Captury Live port (default 2101)")
    ap.add_argument("--osc-host", default="127.0.0.1",
                    help="OSC destination host (default 127.0.0.1)")
    ap.add_argument("--osc-port", type=int, default=7000,
                    help="OSC destination port (default 7000)")
    ap.add_argument("--exe", default="bridge.exe",
                    help="Path to bridge.exe (default ./bridge.exe)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    bridge = Bridge(
        exe_path=args.exe,
        captury_host=args.captury_host,
        captury_port=args.captury_port,
        osc_host=args.osc_host,
        osc_port=args.osc_port,
        verbose=args.verbose,
    )
    bridge.run()


if __name__ == "__main__":
    main()
